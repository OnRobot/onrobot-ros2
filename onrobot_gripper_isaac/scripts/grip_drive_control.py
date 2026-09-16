"""Simulation-only contact preload for a symmetric linear jaw drive.

These helpers set a position-servo reference, not a hardware force command.
They never change gains, effort ceilings, joint limits, or rigid-body poses.
"""

from collections import deque
import hashlib
import math
from pathlib import Path


def script_manifest(runner):
    """Hash the runner and the local modules that define these experiments."""
    runner = Path(runner).resolve()
    names = (runner.name, 'grip_drive_control.py', 'run_payload_retention_test.py',
             'isaac_model_contract.py', 'isaac_runtime_compat.py', 'showcase_branding.py')
    return {name: hashlib.sha256((runner.parent / name).read_bytes()).hexdigest()
            for name in sorted(set(names))}


def linear_preload_target(position_m, stiffness_n_m, generalized_force_n):
    """Request a spring compression below the observed contact position.

    The physical joint limits still constrain motion. For symmetric jaws the
    generalized load is the sum of the two pad normals, not force per pad.
    This reference alone is not a measured or calibrated force guarantee.
    """
    if (not all(math.isfinite(v) for v in (position_m, stiffness_n_m, generalized_force_n))
            or stiffness_n_m <= 0 or generalized_force_n <= 0):
        raise ValueError('preload requires finite position and positive stiffness/force')
    result = position_m - generalized_force_n / stiffness_n_m
    if not math.isfinite(result):
        raise ValueError('preload reference is not finite')
    return result


def hold_control(model, profile, requested=None):
    """Resolve an explicit test-control policy; angular drives need a Jacobian."""
    mode = requested if requested is not None else profile.get('hold_control', 'position')
    if mode not in ('position', 'preload-position'):
        raise ValueError(f'unsupported hold control: {mode}')
    if mode == 'preload-position' and model not in ('2fg7', '2fg14'):
        raise ValueError('linear contact preload is only supported for 2FG models')
    return mode


class PadContactObserver:
    """Require recent bilateral contact before applying a servo preload.

    Ordinary PhysX contact impulses are sufficient for *contact presence*.
    They are not a complete contact wrench or a calibrated grip measurement.
    Each explicit physics step starts a new sample; absent reports mean no
    contact, not reuse of the last sample. Callback errors fail the run.
    """

    def __init__(self, block_path, pad_paths, dt, *, capture_vectors=False):
        if (len(pad_paths) != 2 or len(set(pad_paths)) != 2 or
                block_path in pad_paths or not math.isfinite(dt) or dt <= 0):
            raise ValueError('contact observer needs two distinct pads and a positive timestep')
        self.block_path = block_path
        self.pad_paths = tuple(pad_paths)
        self.dt = dt
        self.normals = [0.0, 0.0]
        self.history = deque(maxlen=max(2, math.ceil(.05 / dt)))
        self.error = None
        self.subscription = None
        self.capture_vectors = capture_vectors
        self.normal_vectors = {}
        self.friction_vectors = {}
        self.points = []

    def begin_step(self):
        self.check_error()
        self.normals = [0.0, 0.0]
        self.normal_vectors = {}
        self.friction_vectors = {}
        self.points = []

    def _vector(self, destination, other, body0, impulse):
        if len(impulse) != 3 or not all(math.isfinite(v) for v in impulse):
            raise ValueError('non-finite or malformed contact impulse')
        sign = 1.0 if body0 == self.block_path else -1.0
        previous = destination.setdefault(other, [0.0, 0.0, 0.0])
        for i, value in enumerate(impulse):
            previous[i] += sign * value / self.dt
        if not all(math.isfinite(v) for v in previous):
            raise ValueError('non-finite contact force sum')

    def observe(self, body0, body1, impulse, normal, position=None):
        if self.block_path not in (body0, body1):
            return
        other = body1 if body0 == self.block_path else body0
        if other not in self.pad_paths and not self.capture_vectors:
            return
        if (len(impulse) != 3 or len(normal) != 3 or
                not all(math.isfinite(v) for v in (*impulse, *normal))):
            raise ValueError('non-finite or malformed pad contact')
        length = math.hypot(*normal)
        if length < 1e-10:
            raise ValueError('pad contact normal has zero length')
        force = abs(sum(v * n / length for v, n in zip(impulse, normal))) / self.dt
        if not math.isfinite(force):
            raise ValueError('pad contact force is not finite')
        if other in self.pad_paths:
            self.normals[self.pad_paths.index(other)] += force
        if self.capture_vectors:
            self._vector(self.normal_vectors, other, body0, impulse)
            if position is not None:
                if len(position) != 3 or not all(math.isfinite(v) for v in position):
                    raise ValueError('invalid contact position')
                if force > 1e-4 and len(self.points) < 64:
                    self.points.append({'body': other, 'position_m': list(position),
                                        'normal': list(normal), 'normal_force_n': force,
                                        'cube_is_actor0': body0 == self.block_path})

    def observe_friction(self, body0, body1, impulse):
        if self.capture_vectors and self.block_path in (body0, body1):
            other = body1 if body0 == self.block_path else body0
            self._vector(self.friction_vectors, other, body0, impulse)

    def vector_sample(self):
        """Optional full-report observation; ordinary reports omit friction.

        Values use the PhysX actor0-impulse convention, reversed for actor1.
        Verify supported weight in the same experiment before interpretation.
        """
        self.check_error()
        if not self.capture_vectors:
            raise RuntimeError('full contact-vector capture is not enabled')
        return {'normal_vectors_on_block_n': {p: list(v) for p, v in self.normal_vectors.items()},
                'friction_vectors_on_block_n': {p: list(v) for p, v in self.friction_vectors.items()},
                'contact_points': [dict(p) for p in self.points]}

    def check_error(self):
        if self.error is not None:
            raise RuntimeError(f'pad contact observation failed: {self.error}')

    def end_step(self):
        self.check_error()
        self.history.append(tuple(self.normals))

    def require_bilateral_contact(self, threshold_n=.1):
        self.check_error()
        if not math.isfinite(threshold_n) or threshold_n <= 0:
            raise ValueError('contact threshold must be finite and positive')
        if (len(self.history) != self.history.maxlen or
                not all(value > threshold_n for value in self.history[-1]) or
                any(sum(row[i] > threshold_n for row in self.history) /
                    len(self.history) < .75 for i in (0, 1))):
            raise RuntimeError('contact preload requires recent bilateral fingertip contact')
        return {'pad_paths': list(self.pad_paths), 'last_pad_normal_magnitudes_n': list(self.normals),
                'window_s': len(self.history) * self.dt, 'threshold_n': threshold_n,
                'basis': 'ordinary-contact-normal-impulse-presence-only'}

    def subscribe(self, stage):
        """Attach to the test object before starting physics; lazy Isaac imports."""
        from pxr import PhysicsSchemaTools, PhysxSchema
        import omni.physx
        if self.subscription is not None:
            raise RuntimeError('contact observer is already subscribed')
        prim = stage.GetPrimAtPath(self.block_path)
        if not prim.IsValid() or any(not stage.GetPrimAtPath(p).IsValid() for p in self.pad_paths):
            raise RuntimeError('contact observer object or fingertip prim is missing')
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)

        def callback(headers, data, friction_anchors=None):
            try:
                for header in headers:
                    a = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
                    b = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
                    for entry in data[header.contact_data_offset:
                                      header.contact_data_offset + header.num_contact_data]:
                        self.observe(a, b, entry.impulse, entry.normal,
                                     entry.position if self.capture_vectors else None)
                    if friction_anchors is not None:
                        for entry in friction_anchors[header.friction_anchors_offset:
                                                      header.friction_anchors_offset + header.num_friction_anchors_data]:
                            self.observe_friction(a, b, entry.impulse)
            except Exception as error:
                if self.error is None:
                    self.error = f'{type(error).__name__}: {error}'

        interface = omni.physx.get_physx_simulation_interface()
        subscribe = (interface.subscribe_full_contact_report_events if self.capture_vectors
                     else interface.subscribe_contact_report_events)
        self.subscription = subscribe(callback)

    def close(self):
        self.subscription = None


def contact_preload_reference(articulation, joint_index, observer):
    """Read native drive values and return a guarded reference plus provenance."""
    contact = observer.require_bilateral_contact()

    def scalar(array):
        values = array.numpy() if hasattr(array, 'numpy') else array
        return float(values[0][joint_index])

    position = scalar(articulation.get_dof_positions())
    stiffness = scalar(articulation.get_dof_gains()[0])
    maximum_force = scalar(articulation.get_dof_max_efforts())
    target = linear_preload_target(position, stiffness, maximum_force)
    return {'contact_position_m': position, 'drive_reference_m': target,
            'native_stiffness_n_m': stiffness, 'requested_generalized_load_n': maximum_force,
            'nominal_load_per_pad_n': maximum_force / 2, 'contact': contact,
            'basis': 'contact-relative-position-servo-reference-not-calibrated-force-control'}
