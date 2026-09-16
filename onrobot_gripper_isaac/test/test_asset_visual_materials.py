"""Run with Isaac Python or any Python containing USD; no SimulationApp needed."""

import importlib.util
import os
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location(
    'asset_visual_materials',
    Path(__file__).resolve().parents[1] / 'scripts/asset_visual_materials.py')
FINISH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FINISH)

# Independent assembled-part expectations: a test based only on the author's
# anchor list missed the white 2FG plate/fingers. These are mesh/subset paths,
# not material definitions, and therefore exercise actual binding resolution.
ALUMINUM_SURFACES = {
    '2fg7': (
        '/Geometry/base_link/base_link/geo_39_1_1/DefaultMaterial_1',
        '/Geometry/base_link/base_link/geo_39_1_1/Mtl10',
        '/Geometry/right_finger_base_link/finger_base_link/finger_base_link',
        '/Geometry/left_finger_base_link/finger_base_link/finger_base_link',
        '/Geometry/right_fingertip_link/silicone_fingertip_link/geo_1_1_1/DefaultMaterial_1_1',
        '/Geometry/left_fingertip_link/silicone_fingertip_link/geo_1_1_1/DefaultMaterial_1_1',
    ),
    '2fg14': (
        '/Geometry/base_link/base_link/geo_28/geo_28',
        '/Geometry/right_finger_base_link/finger_base_link/finger_base_link',
        '/Geometry/left_finger_base_link/finger_base_link/finger_base_link',
        '/Geometry/right_fingertip_link/fingertip_link/geo_0/geo_0_1',
        '/Geometry/left_fingertip_link/fingertip_link/geo_0/geo_0_1',
    ),
}
PRESERVED_SURFACES = {
    '2fg7': (
        '/Geometry/base_link/base_link/geo_39_1_1/texture',  # shared white label
        '/Geometry/base_link/base_link/geo_39_1_1/Mtl13',  # bellows
        '/Geometry/right_fingertip_link/silicone_fingertip_link/geo_1_1_1/Mtl1',
        '/Geometry/left_fingertip_link/silicone_fingertip_link/geo_1_1_1/Mtl1',
    ),
    '2fg14': (
        '/Geometry/base_link/base_link/geo_12/geo_12',  # white label
        '/Geometry/base_link/base_link/geo_29/geo_29',  # dark cover
        '/Geometry/right_fingertip_link/fingertip_link/geo_1/geo_1_1',
        '/Geometry/left_fingertip_link/fingertip_link/geo_1/geo_1_1',
    ),
}
try:
    from pxr import Sdf, Usd, UsdPhysics, UsdShade
except ImportError:
    Usd = None


@unittest.skipIf(Usd is None, 'requires USD Python (run with Isaac Python)')
class VisualMaterialTests(unittest.TestCase):
    @staticmethod
    def define_preview_material(stage, path, color, metallic, roughness, opacity):
        material = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + '/PreviewSurface')
        shader.CreateIdAttr('UsdPreviewSurface')
        values = (
            ('diffuseColor', Sdf.ValueTypeNames.Color3f, color),
            ('metallic', Sdf.ValueTypeNames.Float, metallic),
            ('roughness', Sdf.ValueTypeNames.Float, roughness),
            ('opacity', Sdf.ValueTypeNames.Float, opacity),
        )
        for name, value_type, value in values:
            material.CreateInput(name, value_type).Set(value)
            shader.CreateInput(name, value_type).ConnectToSource(material.GetInput(name))
        material.CreateSurfaceOutput().ConnectToSource(
            shader.CreateOutput('surface', Sdf.ValueTypeNames.Token))
        return material

    def fixture(self, root, model):
        payloads = root / 'payloads'
        payloads.mkdir()
        materials = Usd.Stage.CreateNew(str(payloads / 'materials.usda'))
        original = UsdShade.Material.Define(materials, '/Materials/OriginalWhite')
        original.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set((1, 1, 1))
        rubber = UsdShade.Material.Define(materials, '/Materials/Rubber')
        rubber.GetPrim().CreateAttribute('physics:staticFriction', Sdf.ValueTypeNames.Float).Set(0.6)
        self.define_preview_material(
            materials, '/Materials/Cover', (0.05087609, 0.05087609, 0.05087609),
            0.0, 0.5, 1.0)
        if model in FINISH.RG_PAD_MATERIAL_ANCHORS:
            for index in range(len(FINISH.RG_PAD_MATERIAL_ANCHORS[model])):
                self.define_preview_material(
                    materials, f'/Materials/Pad{index}', (0.2 + index * 0.1, 0.2, 0.2),
                    0.1, 0.3, 1.0)
        materials.GetRootLayer().Save()
        instances = Usd.Stage.CreateNew(str(payloads / 'instances.usda'))
        for path in (*FINISH.MATERIAL_ANCHORS[model], '/Instances/Label/VisualMaterials/White'):
            material = UsdShade.Material.Define(instances, path)
            material.GetPrim().GetReferences().AddReference(
                './materials.usda', '/Materials/OriginalWhite')
        for path in FINISH.RG_COVER_MATERIAL_ANCHORS.get(model, ()):
            material = UsdShade.Material.Define(instances, path)
            material.GetPrim().GetReferences().AddReference(
                './materials.usda', '/Materials/Cover')
        for index, path in enumerate(FINISH.RG_PAD_MATERIAL_ANCHORS.get(model, ())):
            material = UsdShade.Material.Define(instances, path)
            material.GetPrim().GetReferences().AddReference(
                './materials.usda', f'/Materials/Pad{index}')
        instances.GetRootLayer().Save()
        return root / ('onrobot_' + model + '.usda')

    def test_all_models_keep_white_labels_and_physics_material(self):
        for model in FINISH.MATERIAL_ANCHORS:
            with self.subTest(model=model), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                entry = self.fixture(root, model)
                FINISH.author_housing_appearance(entry, model)
                instances = Usd.Stage.Open(str(root / 'payloads/instances.usda'))
                for path in FINISH.MATERIAL_ANCHORS[model]:
                    material = UsdShade.Material(instances.GetPrimAtPath(path))
                    for actual, expected in zip(material.GetInput('diffuseColor').Get(), FINISH.ALUMINUM_COLOR):
                        self.assertAlmostEqual(actual, expected, places=6)
                    shader, _, _ = material.ComputeSurfaceSource()
                    self.assertEqual(shader.GetIdAttr().Get(), 'UsdPreviewSurface')
                    for name, expected in (('metallic', FINISH.ALUMINUM_METALLIC),
                                           ('roughness', FINISH.ALUMINUM_ROUGHNESS),
                                           ('opacity', 1.0)):
                        self.assertAlmostEqual(material.GetInput(name).Get(), expected, places=6)
                        source, source_name, _ = shader.GetInput(name).GetConnectedSource()
                        self.assertEqual(source.GetPrim(), material.GetPrim())
                        self.assertEqual(source_name, name)
                label = UsdShade.Material(instances.GetPrimAtPath('/Instances/Label/VisualMaterials/White'))
                self.assertEqual(tuple(label.GetInput('diffuseColor').Get()), (1, 1, 1))
                materials = Usd.Stage.Open(str(root / 'payloads/materials.usda'))
                self.assertAlmostEqual(materials.GetPrimAtPath('/Materials/Rubber').GetAttribute(
                    'physics:staticFriction').Get(), 0.6, places=6)

    def test_repeated_authoring_is_byte_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entry = self.fixture(root, 'rg6')
            FINISH.author_housing_appearance(entry, 'rg6')
            before = {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()}
            FINISH.author_housing_appearance(entry, 'rg6')
            self.assertEqual(before, {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()})

    def test_rg_pad_visual_inputs_match_cover_and_preserve_contact_material(self):
        for model in FINISH.RG_PAD_MATERIAL_ANCHORS:
            with self.subTest(model=model), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                entry = self.fixture(root, model)
                FINISH.author_housing_appearance(entry, model)
                instances = Usd.Stage.Open(str(root / 'payloads/instances.usda'))
                cover_path = FINISH.RG_COVER_MATERIAL_ANCHORS[model][0]
                cover = UsdShade.Material(instances.GetPrimAtPath(cover_path))
                cover_values = {name: cover.GetInput(name).Get()
                                for name, _ in FINISH.VISUAL_INPUTS}
                for path in FINISH.RG_PAD_MATERIAL_ANCHORS[model]:
                    material = UsdShade.Material(instances.GetPrimAtPath(path))
                    shader, _, _ = material.ComputeSurfaceSource()
                    self.assertEqual(shader.GetIdAttr().Get(), 'UsdPreviewSurface')
                    for name, _ in FINISH.VISUAL_INPUTS:
                        actual = material.GetInput(name).Get()
                        expected = cover_values[name]
                        if name == 'diffuseColor':
                            self.assertEqual(tuple(actual), tuple(expected))
                        else:
                            self.assertAlmostEqual(actual, expected, places=6)
                        source, source_name, _ = shader.GetInput(name).GetConnectedSource()
                        self.assertEqual(source_name, name)
                        self.assertTrue(source)
                materials = Usd.Stage.Open(str(root / 'payloads/materials.usda'))
                self.assertAlmostEqual(materials.GetPrimAtPath('/Materials/Rubber').GetAttribute(
                    'physics:staticFriction').Get(), 0.6, places=6)

    def test_missing_rg_pad_anchor_fails_without_saving_either_layer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entry = self.fixture(root, 'rg6')
            instances = Usd.Stage.Open(str(root / 'payloads/instances.usda'))
            instances.RemovePrim(FINISH.RG_PAD_MATERIAL_ANCHORS['rg6'][-1])
            instances.GetRootLayer().Save()
            before = {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()}
            with self.assertRaisesRegex(RuntimeError, 'pad material anchor changed'):
                FINISH.author_housing_appearance(entry, 'rg6')
            self.assertEqual(before, {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()})

    def test_missing_anchor_fails_without_saving_either_layer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entry = self.fixture(root, 'rg2')
            instances = Usd.Stage.Open(str(root / 'payloads/instances.usda'))
            instances.RemovePrim(FINISH.MATERIAL_ANCHORS['rg2'][-1])
            instances.GetRootLayer().Save()
            before = {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()}
            with self.assertRaisesRegex(RuntimeError, 'anchors changed'):
                FINISH.author_housing_appearance(entry, 'rg2')
            self.assertEqual(before, {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()})

    def test_hybrid_visual_physics_material_is_rejected_without_saving(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entry = self.fixture(root, '2fg7')
            instances = Usd.Stage.Open(str(root / 'payloads/instances.usda'))
            prim = instances.GetPrimAtPath(FINISH.MATERIAL_ANCHORS['2fg7'][0])
            UsdPhysics.MaterialAPI.Apply(prim).CreateStaticFrictionAttr().Set(0.6)
            instances.GetRootLayer().Save()
            before = {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()}
            with self.assertRaisesRegex(RuntimeError, 'physics material data'):
                FINISH.author_housing_appearance(entry, '2fg7')
            self.assertEqual(before, {p.name: p.read_bytes() for p in (root / 'payloads').iterdir()})


@unittest.skipIf(Usd is None, 'requires USD Python (run with Isaac Python)')
class ComposedVisualMaterialTests(unittest.TestCase):
    def test_two_finger_metal_surfaces_and_preserved_details(self):
        # Optional override supports offline native-Isaac staging. With no
        # override, use the same content-pinned repository as installed tools.
        override = os.environ.get('ONROBOT_VISUAL_TEST_ASSETS')
        if override:
            assets = Path(override)
        else:
            spec = importlib.util.spec_from_file_location(
                'visual_test_contract', Path(__file__).resolve().parents[1] /
                'scripts/isaac_model_contract.py')
            contract = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(contract)
            assets = contract.asset_repository_root(Path(__file__).resolve().parents[1]) / 'assets'
        for model in ALUMINUM_SURFACES:
            for mounted in (False, True):
                with self.subTest(model=model, mounted=mounted):
                    entry = assets / model / ('onrobot_' + model + '.usda')
                    self.assertTrue(entry.is_file(), str(entry))
                    root = '/Assembly/Tools/Gripper' if mounted else '/onrobot_' + model
                    if mounted:
                        stage = Usd.Stage.CreateInMemory()
                        stage.DefinePrim(root, 'Xform').GetReferences().AddReference(str(entry))
                    else:
                        stage = Usd.Stage.Open(str(entry))
                    self.assertFalse(stage.GetCompositionErrors())
                    for path in (*ALUMINUM_SURFACES[model], *PRESERVED_SURFACES[model]):
                        prim = stage.GetPrimAtPath(root + path)
                        self.assertTrue(prim.IsValid(), path)
                        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                        self.assertTrue(bool(material), path)
                        is_aluminum = any(str(spec.path) == FINISH.MATERIAL_PATH
                                          for spec in material.GetPrim().GetPrimStack())
                        self.assertEqual(is_aluminum, path in ALUMINUM_SURFACES[model], path)
                        if is_aluminum:
                            for actual, expected in zip(material.GetInput('diffuseColor').Get(),
                                                        FINISH.ALUMINUM_COLOR):
                                self.assertAlmostEqual(actual, expected, places=6)
                            shader, _, _ = material.ComputeSurfaceSource()
                            self.assertEqual(shader.GetIdAttr().Get(), 'UsdPreviewSurface')


if __name__ == '__main__':
    unittest.main()
