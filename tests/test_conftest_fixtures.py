"""
Test root conftest.py fixtures

These tests validate session-scoped fixtures from tests/conftest.py
"""



def test_aircraft_list_fixture(aircraft_list):
    """Test that aircraft_list fixture provides aircraft."""
    # Should have aircraft available
    assert aircraft_list is not None
    assert len(aircraft_list) > 0

    # Should contain common aircraft like c172
    aircraft_names_lower = [a.lower() for a in aircraft_list]
    assert any("c172" in name for name in aircraft_names_lower)


def test_script_list_fixture(script_list):
    """Test that script_list fixture provides scripts."""
    # Should have scripts available
    assert script_list is not None
    assert len(script_list) > 0

    # Should contain some scripts
    assert any("c172" in s.lower() or "ball" in s.lower() for s in script_list)
