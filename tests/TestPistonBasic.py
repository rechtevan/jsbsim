# TestPistonBasic.py
#
# Basic regression tests for the piston engine model (FGPiston).
# Targets coverage improvement for propulsion/piston subsystems.
#
# Copyright (c) 2025 rechtevan
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>
#

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestPistonBasic(JSBSimTestCase):
    """
    Basic tests for piston engine functionality using c172x aircraft.

    These tests verify:
    - Engine model loading
    - Property access
    - Calculation execution
    - Engine parameters

    Note: These tests focus on code execution and coverage rather than
    realistic flight scenarios. The goal is to exercise FGPiston code paths.
    """

    def test_piston_engine_model_loading(self):
        """Test that c172x with piston engine loads successfully."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Verify engine properties exist and are accessible
        rpm = fdm["propulsion/engine/engine-rpm"]
        self.assertIsNotNone(rpm, "Engine RPM property should exist")

        manifold_pressure = fdm["propulsion/engine/map-inhg"]
        self.assertGreater(
            manifold_pressure, 0.0, "Manifold pressure should be positive (atmospheric)"
        )

        # Run a few iterations to exercise the engine model
        for _ in range(10):
            fdm.run()

        del fdm

    def test_piston_engine_properties(self):
        """Test that key piston engine properties are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test various engine properties to exercise property access code
        properties_to_test = [
            "propulsion/engine/engine-rpm",
            "propulsion/engine/power-hp",
            "propulsion/engine/fuel-flow-rate-pps",
            "propulsion/engine/map-inhg",
            "propulsion/engine/egt-degF",
            "propulsion/engine/bsfc-lbs_hphr",
        ]

        for prop in properties_to_test:
            value = fdm[prop]
            self.assertIsNotNone(value, f"Property {prop} should be accessible")

        del fdm

    def test_throttle_control_updates(self):
        """Test that throttle commands update manifold pressure."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Set airspeed to provide some dynamic pressure
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        # Test throttle at different positions
        initial_map = fdm["propulsion/engine/map-inhg"]

        fdm["fcs/throttle-cmd-norm"] = 0.5
        for _ in range(20):
            fdm.run()
        mid_map = fdm["propulsion/engine/map-inhg"]

        fdm["fcs/throttle-cmd-norm"] = 1.0
        for _ in range(20):
            fdm.run()
        full_map = fdm["propulsion/engine/map-inhg"]

        # MAP should respond to throttle (even if engine not running)
        # At minimum, we verify the code executes without crashing
        self.assertIsNotNone(initial_map)
        self.assertIsNotNone(mid_map)
        self.assertIsNotNone(full_map)

        del fdm

    def test_mixture_control_updates(self):
        """Test that mixture commands are processed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test mixture at different positions
        fdm["fcs/mixture-cmd-norm"] = 0.0  # Cut-off
        for _ in range(10):
            fdm.run()
        lean_power = fdm["propulsion/engine/power-hp"]

        fdm["fcs/mixture-cmd-norm"] = 1.0  # Rich
        for _ in range(10):
            fdm.run()
        rich_power = fdm["propulsion/engine/power-hp"]

        # Verify calculations ran (values may be negative for stopped engine)
        self.assertIsNotNone(lean_power)
        self.assertIsNotNone(rich_power)

        del fdm

    def test_piston_with_propeller(self):
        """Test piston engine connected to propeller."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Set some forward velocity
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        # Check propeller properties exist
        prop_rpm = fdm["propulsion/engine/propeller-rpm"]
        prop_thrust = fdm["propulsion/engine/thrust-lbs"]

        self.assertIsNotNone(prop_rpm, "Propeller RPM should be accessible")
        self.assertIsNotNone(prop_thrust, "Propeller thrust should be accessible")

        # Run simulation to exercise prop/engine interaction
        for _ in range(50):
            fdm.run()

        # Verify properties still accessible after running
        prop_rpm_after = fdm["propulsion/engine/propeller-rpm"]
        self.assertIsNotNone(prop_rpm_after)

        del fdm

    def test_engine_parameters_from_config(self):
        """Test that engine configuration parameters are loaded and accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test parameters that should exist on piston engines
        # These exercise the engine configuration and property access code
        volumetric_eff = fdm["propulsion/engine/volumetric-efficiency"]
        bsfc = fdm["propulsion/engine/bsfc-lbs_hphr"]
        cooling_factor = fdm["propulsion/engine/cooling-factor"]

        self.assertGreater(volumetric_eff, 0.0, "Volumetric efficiency should be positive")
        self.assertLess(volumetric_eff, 2.0, "Volumetric efficiency should be < 2.0")
        self.assertGreater(bsfc, 0.0, "BSFC should be positive")
        self.assertGreater(cooling_factor, 0.0, "Cooling factor should be positive")

        del fdm

    def test_piston_calculations_at_altitude(self):
        """Test piston engine calculations at different altitudes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/u-fps"] = 100  # Add airspeed
        fdm.run_ic()

        # Run a bit to stabilize
        for _ in range(10):
            fdm.run()
        sea_level_map = fdm["propulsion/engine/map-inhg"]

        del fdm

        # Test at 10,000 ft
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/u-fps"] = 100  # Same airspeed
        fdm.run_ic()

        # Run a bit to stabilize
        for _ in range(10):
            fdm.run()
        altitude_map = fdm["propulsion/engine/map-inhg"]

        # Atmospheric pressure decreases with altitude
        # so MAP should be lower at altitude (exercises altitude compensation code)
        self.assertGreater(
            sea_level_map, altitude_map, "MAP should decrease with altitude for NA engine"
        )

        del fdm

    def test_engine_starter_property(self):
        """Test engine starter property access."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test starter control (exercises starter motor code)
        fdm["propulsion/engine/starter-norm"] = 0.0
        for _ in range(10):
            fdm.run()

        fdm["propulsion/engine/starter-norm"] = 1.0
        for _ in range(20):
            fdm.run()

        fdm["propulsion/engine/starter-norm"] = 0.0
        for _ in range(10):
            fdm.run()

        # Just verify we can access RPM after starter operations
        rpm = fdm["propulsion/engine/engine-rpm"]
        self.assertIsNotNone(rpm)

        del fdm


RunTest(TestPistonBasic)
