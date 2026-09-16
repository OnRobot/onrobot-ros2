"""Compare actual RG pad surfaces, including subsets and mounted references."""
import importlib.util
import os
from pathlib import Path
import unittest

try:
    from pxr import Usd, UsdShade
except ImportError:
    Usd = None


@unittest.skipIf(Usd is None, 'requires USD Python (run with Isaac Python)')
class RgFingertipMaterials(unittest.TestCase):
    def _visual_signature(self, material, path):
        shader, _, _ = material.ComputeSurfaceSource()
        self.assertTrue(bool(shader), path)
        self.assertEqual(shader.GetIdAttr().Get(), 'UsdPreviewSurface', path)
        signature = {}
        for name in ('diffuseColor', 'metallic', 'roughness', 'opacity'):
            source, source_name, _ = shader.GetInput(name).GetConnectedSource()
            self.assertEqual(source_name, name, path)
            self.assertTrue(bool(source), path)
            value = material.GetInput(name).Get()
            signature[name] = tuple(value) if name == 'diffuseColor' else value
        return signature

    def test_rg2_and_rg6_pad_finishes_match_without_losing_physics_binding(self):
        override = os.environ.get('ONROBOT_VISUAL_TEST_ASSETS')
        if override:
            assets = Path(override)
        else:
            package = Path(__file__).resolve().parents[1]
            spec = importlib.util.spec_from_file_location(
                'rg_visual_contract', package / 'scripts/isaac_model_contract.py')
            contract = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(contract)
            assets = contract.asset_repository_root(package) / 'assets'
        # RG2 stores both surfaces as face subsets; RG6 uses separate meshes.
        surfaces = {
            'rg2': ('geo_1_1/Mtl_1', 'geo_1_1/texture_2'),
            'rg6': ('geo_0/geo_0_1', 'geo_1/geo_1_1'),
        }
        for mounted in (False, True):
            signatures = {}
            for model in surfaces:
                root = '/Assembly/Tools/Gripper' if mounted else '/onrobot_' + model
                entry = assets / model / ('onrobot_' + model + '.usda')
                self.assertTrue(entry.is_file(), str(entry))
                if mounted:
                    stage = Usd.Stage.CreateInMemory()
                    stage.DefinePrim(root, 'Xform').GetReferences().AddReference(str(entry))
                else:
                    stage = Usd.Stage.Open(str(entry))
                self.assertFalse(stage.GetCompositionErrors())
                for side in ('left', 'right'):
                    base = root + '/Geometry/' + side + '_finger_tip_link'
                    cover_link = stage.GetPrimAtPath(
                        root + '/Geometry/' + side + '_finger_cover_link')
                    self.assertTrue(cover_link.IsValid(), str(cover_link.GetPath()))
                    cover_materials = [UsdShade.Material(prim) for prim in Usd.PrimRange(
                        cover_link, Usd.TraverseInstanceProxies())
                                       if prim.IsA(UsdShade.Material) and prim.GetName() == 'cover_grey']
                    self.assertEqual(len(cover_materials), 1, str(cover_link.GetPath()))
                    cover_signature = self._visual_signature(
                        cover_materials[0], str(cover_materials[0].GetPath()))
                    for surface, suffix in enumerate(surfaces[model]):
                        prim = stage.GetPrimAtPath(base + '/fingertip_standard_link/' + suffix)
                        self.assertTrue(prim.IsValid(), str(prim.GetPath()))
                        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                        self.assertTrue(bool(material))
                        signature = self._visual_signature(material, str(prim.GetPath()))
                        with self.subTest(mounted=mounted, model=model, side=side,
                                          surface=surface, check='cover-match'):
                            self.assertEqual(signature, cover_signature)
                        signatures[model, side, surface] = signature
                    targets = stage.GetPrimAtPath(base).GetRelationship(
                        'material:binding:physics').GetTargets()
                    self.assertEqual(list(map(str, targets)), [root + '/Physics/fingertip_physics_material'])
                    physics = stage.GetPrimAtPath(targets[0])
                    self.assertAlmostEqual(physics.GetAttribute('physics:staticFriction').Get(), 0.6, places=6)
                    self.assertAlmostEqual(physics.GetAttribute('physics:dynamicFriction').Get(), 0.5, places=6)
            for side in ('left', 'right'):
                for surface in (0, 1):
                    with self.subTest(mounted=mounted, side=side, surface=surface):
                        self.assertEqual(signatures['rg2', side, surface], signatures['rg6', side, surface])


if __name__ == '__main__':
    unittest.main()
