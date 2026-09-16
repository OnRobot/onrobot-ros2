"""Independent assembled DAE/STL check of the RG standard rubber gap."""
import pytest

from rg_mesh_geometry import expanded_model, gap


@pytest.mark.parametrize('kind', ['visual', 'collision'])
@pytest.mark.parametrize('model,q,width', [
    ('rg2', 0.08093218995, 0.0),
    ('rg2', 0.17217257930, 0.010),
    ('rg2', 0.26448979067, 0.020),
    ('rg2', 1.31360126526, 0.1011),
    ('rg6', 0.06252431382, 0.0),
    ('rg6', 0.12508781087, 0.010),
    ('rg6', 0.18793760962, 0.020),
    ('rg6', 1.29784876444, 0.150),
])
def test_stock_rubber_mesh_gap(model, q, width, kind):
    """Measure transformed mesh surfaces, not a duplicate driver equation."""
    assert gap(model, q, kind=kind) == pytest.approx(width, abs=1e-6)


@pytest.mark.parametrize('model,upper', [
    ('rg2', 1.3136012652574625), ('rg6', 1.2978487644385668),
])
def test_geometry_uses_source_without_installed_model_packages(monkeypatch, model, upper):
    """The entry's package include must not select an old installed macro."""
    monkeypatch.setenv('AMENT_PREFIX_PATH', '')
    tree = expanded_model(model)
    limit = tree.find("joint[@name='finger_joint']/limit")
    assert float(limit.get('upper')) == pytest.approx(upper, abs=1e-12)
