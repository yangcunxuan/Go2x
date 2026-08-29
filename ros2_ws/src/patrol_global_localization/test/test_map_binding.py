"""Map binding / coordinate version tests (P0 audit: overwrite staleness)."""
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
from patrol_global_localization.navigation_gate import (  # noqa: E402
    map_binding_violation)


def test_matching_binding_passes():
    point = {'map_name': 'factory_a', 'coord_version': 'v1'}
    assert map_binding_violation(point, 'factory_a',
                                 {'coord_version': 'v1'}) is None


def test_wrong_map_blocks():
    err = map_binding_violation({'map_name': 'other', 'coord_version': 'v1'},
                                'factory_a', {'coord_version': 'v1'})
    assert err and '不属于' in err


def test_stale_coord_version_blocks():
    """Same map name after a rebuild: old coord_version must be rejected."""
    err = map_binding_violation({'map_name': 'factory_a', 'coord_version': 'old'},
                                'factory_a', {'coord_version': 'new'})
    assert err and '坐标版本' in err


def test_unbound_point_blocks():
    err = map_binding_violation({'map_name': '__live__'}, 'factory_a',
                                {'coord_version': 'v1'})
    assert err


def test_none_metadata_blocks_legacy_map():
    """Fail-closed: legacy maps without coord_version need one re-save."""
    err = map_binding_violation({'map_name': 'factory_a'}, 'factory_a', None)
    assert err and '旧格式' in err
    err = map_binding_violation({'map_name': 'factory_a', 'coord_version': 'whatever'},
                                'factory_a', None)
    assert err and '旧格式' in err
