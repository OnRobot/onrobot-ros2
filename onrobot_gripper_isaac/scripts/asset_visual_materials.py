"""Portable visual-only finish authoring for the four stock gripper assets.

Uses USD only; no ROS or simulator startup is required by this module. Explicit
anchors keep imported shared white label materials out of the housing override.
"""

from pathlib import Path
import math


ALUMINUM_COLOR = (0.55, 0.57, 0.60)
ALUMINUM_METALLIC = 0.6
ALUMINUM_ROUGHNESS = 0.4
MATERIAL_PATH = '/Materials/OnRobotAluminumHousing'
MATERIAL_ANCHORS = {
    '2fg7': (
        '/Instances/geo_39_1_1/VisualMaterials/DefaultMaterial_1',  # housing
        '/Instances/geo_39_1_1/VisualMaterials/Mtl10',  # plate below bellows
        '/Instances/finger_base_link/VisualMaterials/material_1',
        '/Instances/finger_base_link_1/VisualMaterials/material_2',
        '/Instances/geo_1_1_1/VisualMaterials/DefaultMaterial_1_1',  # metal tip backing
    ),
    '2fg14': (
        '/Instances/geo_28/VisualMaterials/Mtl',  # housing
        '/Instances/finger_base_link/VisualMaterials/material_1',
        '/Instances/finger_base_link_2/VisualMaterials/material_2',
        '/Instances/geo_0_1/VisualMaterials/Mtl_1',  # both aluminum L fingers
    ),
    'rg2': (
        '/Instances/geo_10_1/VisualMaterials/Mtl4',
        '/Instances/geo_10_1/VisualMaterials/Mtl6',
        '/Instances/geo_8_1/VisualMaterials/Mtl',
        '/Instances/geo_8_1/VisualMaterials/Mtl1_1',
        *tuple(f'/Instances/{side}_{part}/VisualMaterials/body_white'
               for side in ('left', 'right')
               for part in ('outer_proximal_finger_link', 'finger_base_link',
                            'inner_proximal_finger_link')),
    ),
    'rg6': (
        '/Instances/geo_22/VisualMaterials/texture',
        '/Instances/geo_20/VisualMaterials/texture1',
        '/Instances/geo_12_1/VisualMaterials/Mtl_1',
        '/Instances/geo_14_1/VisualMaterials/Mtl2_1',
        *tuple(f'/Instances/{part}/VisualMaterials/body_white'
               for part in ('outer_proximal_finger_link', 'finger_base_link',
                            'inner_proximal_finger_link')),
    ),
}

RG_COVER_MATERIAL_ANCHORS = {
    'rg2': (
        '/Instances/right_finger_cover_link/VisualMaterials/cover_grey',
        '/Instances/left_finger_cover_link/VisualMaterials/cover_grey',
    ),
    'rg6': (
        '/Instances/finger_cover_link/VisualMaterials/cover_grey',
    ),
}
RG_PAD_MATERIAL_ANCHORS = {
    'rg2': (
        '/Instances/geo_1_1/VisualMaterials/Mtl_1',
        '/Instances/geo_1_1/VisualMaterials/texture_2',
    ),
    'rg6': (
        '/Instances/geo_0_1/VisualMaterials/Mtl_2',
        '/Instances/geo_1_1/VisualMaterials/Mtl1_1',
    ),
}
VISUAL_INPUTS = (
    ('diffuseColor', 'Color3f'),
    ('metallic', 'Float'),
    ('roughness', 'Float'),
    ('opacity', 'Float'),
)


def _rg_pad_appearance(instances, model: str, UsdPhysics, UsdShade, Sdf) -> dict:
    """Copy the cover's composed visual shader values onto the RG pad materials.

    These are visual-only material prims in ``instances.usda``. Contact
    materials remain in the Physics layers and are deliberately not authored.
    """
    if model not in RG_PAD_MATERIAL_ANCHORS:
        return {}

    def get_visual_values(path: str, role: str) -> dict:
        prim = instances.GetPrimAtPath(path)
        if not prim or not prim.IsA(UsdShade.Material):
            raise RuntimeError(f'{model} RG {role} material anchor changed: {path}')
        if prim.HasAPI(UsdPhysics.MaterialAPI) or any(
                attr.GetName().startswith(('physics:', 'physxMaterial:'))
                for attr in prim.GetAttributes()):
            raise RuntimeError(f'{model} RG {role} material includes physics data: {path}')
        material = UsdShade.Material(prim)
        shader, _, _ = material.ComputeSurfaceSource()
        if not shader or shader.GetIdAttr().Get() != 'UsdPreviewSurface':
            raise RuntimeError(f'{model} RG {role} material is not a UsdPreviewSurface: {path}')
        values = {}
        for name, _ in VISUAL_INPUTS:
            material_input = material.GetInput(name)
            shader_input = shader.GetInput(name)
            value = material_input.Get() if material_input else None
            if not shader_input or value is None:
                raise RuntimeError(
                    f'{model} RG {role} material is missing visual input {name}: {path}')
            components = tuple(value) if name == 'diffuseColor' else (value,)
            if not all(math.isfinite(float(component)) for component in components):
                raise RuntimeError(
                    f'{model} RG {role} material has non-finite {name}: {path}')
            values[name] = value
        return values

    cover_anchors = RG_COVER_MATERIAL_ANCHORS[model]
    pad_anchors = RG_PAD_MATERIAL_ANCHORS[model]
    cover_values = [get_visual_values(path, 'cover') for path in cover_anchors]
    reference = cover_values[0]
    for path, values in zip(cover_anchors[1:], cover_values[1:]):
        if any(tuple(values[name]) != tuple(reference[name])
               if name == 'diffuseColor' else values[name] != reference[name]
               for name, _ in VISUAL_INPUTS):
            raise RuntimeError(
                f'{model} left/right cover visual materials differ; pad mapping needs review: {path}')
    pad_materials = []
    for path in pad_anchors:
        get_visual_values(path, 'pad')  # validate shader/material before authoring
        pad_materials.append(UsdShade.Material(instances.GetPrimAtPath(path)))

    value_types = {
        'Color3f': Sdf.ValueTypeNames.Color3f,
        'Float': Sdf.ValueTypeNames.Float,
    }
    for material in pad_materials:
        for name, kind in VISUAL_INPUTS:
            material.CreateInput(name, value_types[kind]).Set(reference[name])

    return {
        'cover_anchors': list(cover_anchors),
        'pad_anchors': list(pad_anchors),
        'visual_inputs': {name: list(reference[name]) if name == 'diffuseColor'
                          else reference[name] for name, _ in VISUAL_INPUTS},
    }


def author_housing_appearance(entrypoint: Path, model: str) -> dict:
    """Update only material values and known visual reference anchors.

    Missing anchors fail before writing either layer. Geometry, contact-purpose
    bindings, mass, drives and lighting are not authored here. The isotropic
    preview finish approximates satin aluminum; no measured brush texture is
    claimed. Importer layout changes require an explicit anchor review.
    """
    from pxr import Sdf, Usd, UsdPhysics, UsdShade

    payloads = Path(entrypoint).parent / 'payloads'
    instances = Usd.Stage.Open(str(payloads / 'instances.usda'))
    materials = Usd.Stage.Open(str(payloads / 'materials.usda'))
    if not instances or not materials:
        raise RuntimeError('asset visual layers are missing')
    anchors = [instances.GetPrimAtPath(path) for path in MATERIAL_ANCHORS[model]]
    missing = [path for path, prim in zip(MATERIAL_ANCHORS[model], anchors)
               if not prim or not prim.IsA(UsdShade.Material)]
    if missing:
        raise RuntimeError(f'{model} visual material anchors changed: {missing}')
    hybrid = [str(prim.GetPath()) for prim in anchors
              if prim.HasAPI(UsdPhysics.MaterialAPI) or any(
                  attr.GetName().startswith(('physics:', 'physxMaterial:'))
                  for attr in prim.GetAttributes())]
    if hybrid:
        raise RuntimeError(f'{model} visual anchors contain physics material data: {hybrid}')
    material = UsdShade.Material.Define(materials, MATERIAL_PATH)
    values = (
        ('diffuseColor', Sdf.ValueTypeNames.Color3f, ALUMINUM_COLOR),
        ('metallic', Sdf.ValueTypeNames.Float, ALUMINUM_METALLIC),
        ('roughness', Sdf.ValueTypeNames.Float, ALUMINUM_ROUGHNESS),
        ('opacity', Sdf.ValueTypeNames.Float, 1.0),
    )
    shader = UsdShade.Shader.Define(materials, MATERIAL_PATH + '/PreviewSurface')
    shader.CreateIdAttr('UsdPreviewSurface')
    for name, kind, value in values:
        material.CreateInput(name, kind).Set(value)
        shader.CreateInput(name, kind).ConnectToSource(material.GetInput(name))
    material.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput('surface', Sdf.ValueTypeNames.Token))
    pad_appearance = _rg_pad_appearance(instances, model, UsdPhysics, UsdShade, Sdf)
    for prim in anchors:
        prim.GetReferences().SetReferences([
            Sdf.Reference('./materials.usda', MATERIAL_PATH)])
    materials.GetRootLayer().Save()
    instances.GetRootLayer().Save()
    report = {'housing_material': MATERIAL_PATH,
              'anchors': list(MATERIAL_ANCHORS[model]),
              'diffuse_color': list(ALUMINUM_COLOR),
              'metallic': ALUMINUM_METALLIC, 'roughness': ALUMINUM_ROUGHNESS}
    if pad_appearance:
        report['rg_pad_appearance'] = pad_appearance
    return report
