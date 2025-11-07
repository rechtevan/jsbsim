# __init__.py
#
# Integration Tests Package for JSBSim
#
# This package contains integration tests that exercise multiple JSBSim
# components working together in realistic flight scenarios. These tests
# complement the existing unit tests by validating component interactions
# and overall system behavior.
#
# Test Organization:
# - test_01_aircraft_loading.py: Foundation test for aircraft loading
# - test_02_steady_level_flight.py: Full simulation loop testing
# - test_03_takeoff_sequence.py: Ground to flight transition
# - test_04_coordinated_turn.py: Flight control system testing
# - test_05_landing_approach.py: Descent and landing dynamics
# - test_06_propulsion_dynamics.py: Engine and propulsion testing
# - test_07_aero_coefficients.py: Aerodynamic table interpolation
# - test_08_atmospheric_model.py: Atmosphere model validation
# - test_09_fuel_mass_balance.py: Fuel consumption and CG effects
# - test_10_script_execution.py: Script parsing and event handling
#
# Copyright (c) 2025 JSBSim Development Team
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.

__version__ = "1.0.0"
__author__ = "JSBSim Development Team"
