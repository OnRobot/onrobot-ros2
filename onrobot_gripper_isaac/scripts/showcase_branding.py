#!/usr/bin/env python3
"""Brand assets used by the Isaac Sim presentation harnesses."""

import os
from pathlib import Path
import warnings


ONROBOT_BLUE = (73.0 / 255.0, 157.0 / 255.0, 218.0 / 255.0)
ONROBOT_BLACK = (38.0 / 255.0, 41.0 / 255.0, 43.0 / 255.0)


def _sign_font(size, bold=False):
    """Prefer the brand font, with readable offline fallbacks on headless Kit."""
    from PIL import ImageFont

    suffix = 'Bold' if bold else 'Regular'
    for path in (
            f'/usr/share/fonts/truetype/ubuntu/Ubuntu-{suffix}.ttf',
            '/usr/share/fonts/truetype/ubuntu/Ubuntu-' + ('B.ttf' if bold else 'R.ttf'),
            f'/usr/share/fonts/truetype/ubuntu/UbuntuSans-{suffix}.ttf',
            '/usr/share/fonts/truetype/msttcorefonts/' + ('arialbd.ttf' if bold else 'arial.ttf'),
            f'/usr/share/fonts/truetype/liberation/LiberationSans-{suffix}.ttf',
            f'/usr/share/fonts/truetype/liberation2/LiberationSans-{suffix}.ttf'):
        try:
            font = ImageFont.truetype(path, size)
            if 'liberation' in path:
                warnings.warn('Ubuntu/Arial unavailable: using an engineering font fallback; '
                              'install an approved font before recording marketing material', stacklevel=2)
            return font
        except OSError:
            continue
    # Pillow's scalable fallback is preferable to its old, fixed 10-pixel
    # bitmap font. A missing font must not silently erase the presentation text.
    try:
        warnings.warn('Ubuntu/Arial unavailable: using a Pillow engineering font fallback', stacklevel=2)
        return ImageFont.load_default(size=size)
    except TypeError as error:
        raise RuntimeError('Install an Ubuntu/Arial font or use Pillow >= 10.1 for readable signs') from error


def define_textured_sign(stage, path, texture_path, center, size, facing='X'):
    """Add a visual-only sign facing +X or -Y, with unchanged logo artwork."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    x, y, z = center
    w, h = size[0] / 2, size[1] / 2
    if facing == 'X':
        points = [(x, y-w, z-h), (x, y+w, z-h), (x, y+w, z+h), (x, y-w, z+h)]
    elif facing == '-Y':
        points = [(x-w, y, z-h), (x+w, y, z-h), (x+w, y, z+h), (x-w, y, z+h)]
    else:
        raise ValueError('sign facing must be X or -Y')
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        'st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(
            Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    material = UsdShade.Material.Define(stage, f'{path}_material')
    surface = UsdShade.Shader.Define(stage, f'{path}_material/surface')
    surface.CreateIdAttr('UsdPreviewSurface')
    surface.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.75)
    surface.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0)
    texture = UsdShade.Shader.Define(stage, f'{path}_material/texture')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture_path)))
    texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    texture.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('clamp')
    texture.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('clamp')
    texture.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    reader = UsdShade.Shader.Define(stage, f'{path}_material/reader')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    reader.CreateOutput('result', Sdf.ValueTypeNames.Float2)
    texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    surface.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), 'rgb')
    surface.CreateOutput('surface', Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh.GetPrim()


def create_studio_lighting(stage, root: str):
    """Add neutral fill and a broad key light to an example scene only.

    Directional illumination makes metal edges readable without replacing the
    materials or embedding environment lights in the reusable gripper asset.
    """
    from pxr import Gf, UsdGeom, UsdLux

    fill = UsdLux.DomeLight.Define(stage, f'{root}/fill')
    fill.CreateIntensityAttr().Set(250.0)
    fill.CreateExposureAttr().Set(0.0)
    key = UsdLux.DistantLight.Define(stage, f'{root}/key')
    key.CreateIntensityAttr().Set(1200.0)
    key.CreateAngleAttr().Set(12.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(30.0, 35.0, 0.0))
    return {'fill_intensity': 250.0, 'key_intensity': 1200.0,
            'key_angle_deg': 12.0}


def create_showcase_sign(model: str, payload_mass: float,
                         logo_path: Path) -> Path:
    """Compose a temporary sign while keeping the approved logo unchanged."""
    from PIL import Image, ImageDraw

    if not logo_path.is_file():
        raise RuntimeError(f'approved OnRobot logo is missing: {logo_path}')
    canvas = Image.new('RGB', (1200, 600), (255, 255, 255))
    logo = Image.open(logo_path).convert('RGBA')
    logo.thumbnail((600, 240), Image.Resampling.LANCZOS)
    canvas.paste(logo, (70, 60), logo)

    heading_font = _sign_font(82, bold=True)
    body_font = _sign_font(58)
    draw = ImageDraw.Draw(canvas)
    draw.text((70, 335), model.upper(), fill=(38, 41, 43),
              font=heading_font)
    draw.text((70, 455), f'Payload  {payload_mass:g} kg',
              fill=(126, 135, 142), font=body_font)
    draw.rectangle((0, 582, 1200, 600), fill=(73, 157, 218))

    sign_path = Path('/tmp') / (
        f'onrobot_{model}_physical_showcase_{payload_mass:g}_{os.getpid()}.png')
    canvas.save(sign_path)
    return sign_path


def create_payload_label(payload_mass: float) -> Path:
    """Render a readable mass label for a visual-only workpiece face."""
    from PIL import Image, ImageDraw

    canvas = Image.new('RGB', (640, 220), (223, 228, 232))
    draw = ImageDraw.Draw(canvas)
    draw.text((320, 110), f'{payload_mass:g} kg', font=_sign_font(105, bold=True),
              fill=(38, 41, 43), anchor='mm')
    path = Path('/tmp') / f'onrobot_payload_{payload_mass:g}_{os.getpid()}.png'
    canvas.save(path)
    return path
