#!/usr/bin/env python3
"""Small feature-based compatibility helpers for supported Isaac runtimes."""

import math


def set_position_target(articulation, target, *, dof_indices) -> None:
    """Command a stationary position target after any previous velocity command."""
    # Position drives retain their velocity target when the control mode changes.
    # Clear that feed-forward term without overwriting measured joint velocity.
    articulation.set_dof_velocity_targets(0.0, dof_indices=dof_indices)
    articulation.set_dof_position_targets(target, dof_indices=dof_indices)


def configure_standalone_physics(stage, model: str) -> None:
    """Supply the removed asset scene defaults when no application scene exists."""
    from pxr import UsdPhysics, PhysxSchema

    if any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse()):
        return
    scene = UsdPhysics.Scene.Define(stage, '/PhysicsScene')
    physics = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    if model in ('rg2', 'rg6'):
        physics.CreateEnableStabilizationAttr().Set(True)
        physics.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(8192)
    if model == 'rg6':
        physics.CreateSolveArticulationContactLastAttr().Set(True)


def configure_standalone_view(stage, model_prim_path: str) -> str:
    """Frame a standalone gripper; keep camera and lights in the application scene."""
    from pxr import Gf, Usd, UsdGeom, UsdLux
    from omni.kit.viewport.utility import get_active_viewport

    model = stage.GetPrimAtPath(model_prim_path)
    if not model:
        raise RuntimeError(f'Cannot frame missing model: {model_prim_path}')
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(
            model).ComputeAlignedRange()
    radius = bounds.GetSize().GetLength() * 0.5
    if bounds.IsEmpty() or not math.isfinite(radius) or radius <= 0.0:
        raise RuntimeError(f'Cannot frame empty model: {model_prim_path}')
    center = bounds.GetMidpoint()
    eye = center + Gf.Vec3d(1.4, -2.0, -0.7).GetNormalized() * radius * 5.4
    camera = UsdGeom.Camera.Define(stage, '/OnRobotViewer/Camera')
    camera.CreateFocalLengthAttr(50.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, 100.0))
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(
        Gf.Matrix4d().SetLookAt(eye, center, Gf.Vec3d(0, 0, -1)).GetInverse())
    if not any(prim.HasAPI(UsdLux.LightAPI) for prim in stage.Traverse()):
        UsdLux.DomeLight.Define(
            stage, '/OnRobotViewer/Fill').CreateIntensityAttr(600.0)
        key = UsdLux.DistantLight.Define(stage, '/OnRobotViewer/Key')
        key.CreateIntensityAttr(1800.0)
        UsdGeom.Xformable(key).AddRotateXYZOp().Set((30.0, -35.0, -30.0))
    viewport = get_active_viewport()
    if viewport is not None:
        viewport.camera_path = str(camera.GetPath())
    return str(camera.GetPath())


def enable_extension(name: str) -> None:
    """Enable an extension through the API available in this runtime."""
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        app_utils.enable_extension(name)
    except ModuleNotFoundError:
        from isaacsim.core.utils.extensions import enable_extension as enable
        enable(name)


def is_stage_loading() -> bool:
    """Return whether the current USD context still has pending assets."""
    try:
        import isaacsim.core.experimental.utils.stage as stage_utils
        predicate = getattr(stage_utils, 'is_stage_loading', None)
        if predicate is not None:
            return bool(predicate())
    except ModuleNotFoundError:
        pass
    import omni.usd
    return omni.usd.get_context().get_stage_loading_status()[2] > 0


def setup_simulation(manager, dt: float, device: str = 'cpu') -> float:
    """Set up physics and return the runtime's effective physics step."""
    # These assets and their contact/drive tests use PhysX. New runtimes can
    # start another engine through an enabled extension; choose it explicitly.
    active_engine = getattr(manager, 'get_active_physics_engine', None)
    if active_engine is not None and active_engine() != 'physx':
        switch_engine = getattr(manager, 'switch_physics_engine', None)
        if switch_engine is None or not switch_engine('physx') or active_engine() != 'physx':
            raise RuntimeError('OnRobot simulation requires the PhysX engine')
    physics_scene = None
    physics_scene_api = None
    steps_per_second = None
    setup = getattr(manager, 'setup_simulation', None)
    if setup is not None:
        setup(dt=dt, device=device)
    else:
        # Isaac Sim 5 can open a stage without registering its authored
        # PhysicsScene in SimulationManager. In that state set_physics_dt()
        # is a silent no-op and get_physics_dt() returns the 60 Hz fallback.
        # Register the single authored scene explicitly before initialization.
        try:
            import omni.usd
            from pxr import PhysxSchema
            stage = omni.usd.get_context().get_stage()
            scene_paths = [
                str(prim.GetPath()) for prim in stage.Traverse()
                if prim.GetTypeName() == 'PhysicsScene'
            ] if stage is not None else []
            if len(scene_paths) != 1:
                raise RuntimeError(
                    'expected one authored PhysicsScene, found '
                    f'{scene_paths}')
            physics_scene = scene_paths[0]
            steps_per_second = round(1.0 / dt)
            if not math.isclose(
                    1.0 / steps_per_second, dt,
                    rel_tol=1e-6, abs_tol=1e-9):
                raise RuntimeError(
                    f'physics step {dt} s is not an integral rate')
            physics_scene_api = PhysxSchema.PhysxSceneAPI.Apply(
                stage.GetPrimAtPath(physics_scene))
            physics_scene_api.CreateTimeStepsPerSecondAttr().Set(
                steps_per_second)
            manager.set_default_physics_scene(physics_scene)
        except ModuleNotFoundError:
            # Unit-test doubles exercise the call ordering without Isaac.
            pass
        manager.set_physics_sim_device(device)
        if physics_scene is None:
            manager.set_physics_dt(dt)
        else:
            manager.set_physics_dt(dt, physics_scene=physics_scene)
        manager.initialize_physics()
        if physics_scene is not None:
            # Stage-load callbacks in some Isaac 5 applications rebuild the
            # manager registry during initialization. Re-select the authored
            # scene so the postcondition below reads the same scene we set.
            manager.set_default_physics_scene(physics_scene)
            physics_scene_api.CreateTimeStepsPerSecondAttr().Set(
                steps_per_second)
            manager.set_physics_dt(dt, physics_scene=physics_scene)
    actual = float(manager.get_physics_dt())
    if not math.isfinite(actual) or actual <= 0.0:
        raise RuntimeError(f'Isaac returned invalid physics step {actual}')
    return actual


def play() -> None:
    """Start the timeline without binding callers to an app utility module."""
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        app_utils.play()
    except ModuleNotFoundError:
        import omni.timeline
        omni.timeline.get_timeline_interface().play()


def stop() -> None:
    """Stop the timeline without binding callers to an app utility module."""
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        app_utils.stop()
    except ModuleNotFoundError:
        import omni.timeline
        omni.timeline.get_timeline_interface().stop()


def close_app(app, exit_code: int) -> None:
    """Close SimulationApp across Isaac Sim 5 and 6 signatures."""
    try:
        app.close(exit_code=exit_code)
    except TypeError:
        app.close()
