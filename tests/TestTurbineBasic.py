# TestTurbineBasic.py
#
# Basic regression tests for turbine engine models (FGTurbine, FGTurboprop).
# Targets coverage improvement for propulsion/turbine subsystems.
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


class TestTurbineBasic(JSBSimTestCase):
    """
    Basic tests for turbine engine functionality using 737 and F-16 aircraft.

    These tests verify:
    - Engine model loading (turbine and turboprop)
    - Property access (N1, N2, thrust, fuel flow, etc.)
    - Calculation execution
    - Throttle response
    - Altitude performance
    - Afterburner operation (F-16)

    Note: These tests focus on code execution and coverage rather than
    realistic flight scenarios. The goal is to exercise FGTurbine code paths.
    """

    def test_turbine_engine_loading(self):
        """Test that 737 with turbine engines loads successfully."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")
        fdm.run_ic()

        # Verify engine properties exist and are accessible
        n1 = fdm["propulsion/engine[0]/n1"]
        self.assertIsNotNone(n1, "Engine N1 property should exist")

        n2 = fdm["propulsion/engine[0]/n2"]
        self.assertIsNotNone(n2, "Engine N2 property should exist")

        thrust = fdm["propulsion/engine[0]/thrust-lbs"]
        self.assertIsNotNone(thrust, "Engine thrust property should exist")

        # Set engine running to test actual operation
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["propulsion/engine[1]/set-running"] = 1

        # Run a few iterations to exercise the engine model
        for _ in range(10):
            fdm.run()

        del fdm

    def test_turbine_engine_properties(self):
        """Test that key turbine engine properties are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")
        fdm.run_ic()

        # Test various turbine-specific properties
        # Note: EGT and EPR properties don't exist in JSBSim turbine model
        properties_to_test = [
            "propulsion/engine[0]/n1",
            "propulsion/engine[0]/n2",
            "propulsion/engine[0]/thrust-lbs",
            "propulsion/engine[0]/fuel-flow-rate-pps",
            "propulsion/engine[1]/n1",
            "propulsion/engine[1]/n2",
        ]

        for prop in properties_to_test:
            value = fdm[prop]
            self.assertIsNotNone(value, f"Property {prop} should be accessible")

        del fdm

    def test_turbine_throttle_response(self):
        """Test that throttle commands affect thrust output."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")

        # Set initial conditions with some altitude and speed
        fdm["ic/h-sl-ft"] = 10000.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        # Set engines running
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["propulsion/engine[1]/set-running"] = 1

        # Test throttle at idle
        fdm["fcs/throttle-cmd-norm[0]"] = 0.0
        for _ in range(50):
            fdm.run()
        idle_thrust = fdm["propulsion/engine[0]/thrust-lbs"]
        idle_n1 = fdm["propulsion/engine[0]/n1"]

        # Test throttle at mid setting
        fdm["fcs/throttle-cmd-norm[0]"] = 0.5
        for _ in range(50):
            fdm.run()
        mid_thrust = fdm["propulsion/engine[0]/thrust-lbs"]

        # Test throttle at full
        fdm["fcs/throttle-cmd-norm[0]"] = 1.0
        for _ in range(50):
            fdm.run()
        full_thrust = fdm["propulsion/engine[0]/thrust-lbs"]
        full_n1 = fdm["propulsion/engine[0]/n1"]

        # Verify thrust increases with throttle
        self.assertGreater(
            full_thrust, idle_thrust, "Full throttle should produce more thrust than idle"
        )
        self.assertGreater(full_n1, idle_n1, "Full throttle should produce higher N1 than idle")

        # Verify all values are reasonable (thrust values depend on altitude and speed)
        self.assertGreater(full_thrust, 0.0, "Full throttle should produce thrust")
        self.assertGreater(mid_thrust, 0.0, "Mid throttle should produce thrust")

        # Verify thrust is responsive to throttle changes
        self.assertGreater(mid_thrust, idle_thrust, "Mid throttle should produce more than idle")

        del fdm

    def test_turbine_n1_n2_parameters(self):
        """Test N1 and N2 spool speed parameters."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 200.0
        fdm.run_ic()

        # Set engines running
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["propulsion/engine[1]/set-running"] = 1

        # Set throttle and run to stabilize
        fdm["fcs/throttle-cmd-norm[0]"] = 0.8
        for _ in range(100):
            fdm.run()

        # Check N1 and N2 are within reasonable ranges
        n1 = fdm["propulsion/engine[0]/n1"]
        n2 = fdm["propulsion/engine[0]/n2"]

        self.assertGreater(n1, 0.0, "N1 should be positive")
        self.assertLess(n1, 120.0, "N1 should be less than 120%")
        self.assertGreater(n2, 0.0, "N2 should be positive")
        self.assertLess(n2, 120.0, "N2 should be less than 120%")

        # N2 (high-pressure spool) typically higher than N1 at cruise
        # But this depends on engine design, so we just verify both exist
        self.assertIsNotNone(n1)
        self.assertIsNotNone(n2)

        del fdm

    def test_turbine_fuel_flow(self):
        """Test turbine fuel consumption calculations."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")

        fdm["ic/h-sl-ft"] = 10000.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        # Get initial fuel quantity
        initial_fuel = fdm["propulsion/total-fuel-lbs"]

        # Set engines running
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["propulsion/engine[1]/set-running"] = 1

        # Set throttle to produce thrust
        fdm["fcs/throttle-cmd-norm[0]"] = 0.7
        fdm["fcs/throttle-cmd-norm[1]"] = 0.7

        # Run for several seconds
        for _ in range(200):
            fdm.run()

        # Check fuel flow rate is positive
        fuel_flow_0 = fdm["propulsion/engine[0]/fuel-flow-rate-pps"]
        fuel_flow_1 = fdm["propulsion/engine[1]/fuel-flow-rate-pps"]

        self.assertGreater(fuel_flow_0, 0.0, "Fuel flow should be positive at power")
        self.assertGreater(fuel_flow_1, 0.0, "Fuel flow should be positive at power")

        # Verify fuel has been consumed
        final_fuel = fdm["propulsion/total-fuel-lbs"]
        self.assertLess(final_fuel, initial_fuel, "Fuel should be consumed during operation")

        del fdm

    def test_turbine_at_altitude(self):
        """Test turbine engine performance at different altitudes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm["ic/vc-kts"] = 200.0
        fdm.run_ic()

        # Set engines running
        fdm["propulsion/engine[0]/set-running"] = 1

        fdm["fcs/throttle-cmd-norm[0]"] = 0.8
        for _ in range(50):
            fdm.run()
        sea_level_thrust = fdm["propulsion/engine[0]/thrust-lbs"]

        del fdm

        # Test at 30,000 ft
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")
        fdm["ic/h-sl-ft"] = 30000.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        # Set engines running
        fdm["propulsion/engine[0]/set-running"] = 1

        fdm["fcs/throttle-cmd-norm[0]"] = 0.8
        for _ in range(50):
            fdm.run()
        altitude_thrust = fdm["propulsion/engine[0]/thrust-lbs"]

        # Thrust decreases with altitude due to lower air density
        self.assertGreater(
            sea_level_thrust, altitude_thrust, "Thrust should decrease with altitude"
        )

        # Both should still produce positive thrust
        self.assertGreater(sea_level_thrust, 0.0)
        self.assertGreater(altitude_thrust, 0.0)

        del fdm

    def test_turbine_running_state(self):
        """Test turbine engine set-running property and state changes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")

        fdm["ic/h-sl-ft"] = 10000.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        # Engine should not be running initially
        initial_running = fdm["propulsion/engine[0]/set-running"]
        self.assertEqual(initial_running, 0.0, "Engine should not be running initially")

        # Set engine to running state
        fdm["propulsion/engine[0]/set-running"] = 1

        # Set moderate throttle
        fdm["fcs/throttle-cmd-norm[0]"] = 0.6
        for _ in range(50):
            fdm.run()

        # Check that engine is now producing thrust
        thrust = fdm["propulsion/engine[0]/thrust-lbs"]
        n1 = fdm["propulsion/engine[0]/n1"]
        n2 = fdm["propulsion/engine[0]/n2"]

        self.assertGreater(thrust, 0.0, "Running engine should produce thrust")
        self.assertGreater(n1, 0.0, "Running engine should have positive N1")
        self.assertGreater(n2, 0.0, "Running engine should have positive N2")

        del fdm

    def test_afterburner_operation(self):
        """Test afterburner functionality using F-16."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")

        fdm["ic/h-sl-ft"] = 15000.0
        fdm["ic/vc-kts"] = 400.0
        fdm.run_ic()

        # Set engine running
        fdm["propulsion/engine/set-running"] = 1

        # Military power (no afterburner)
        fdm["fcs/throttle-cmd-norm"] = 0.95
        for _ in range(50):
            fdm.run()
        mil_thrust = fdm["propulsion/engine/thrust-lbs"]

        # Full afterburner
        fdm["fcs/throttle-cmd-norm"] = 1.0
        for _ in range(50):
            fdm.run()
        ab_thrust = fdm["propulsion/engine/thrust-lbs"]

        # Afterburner should increase thrust
        # Note: Augmentation property doesn't exist as a readable property,
        # but thrust should increase at full throttle
        self.assertGreater(
            ab_thrust, mil_thrust, "Afterburner should produce more thrust than military power"
        )

        # Verify both produce significant thrust
        self.assertGreater(mil_thrust, 1000.0, "Military power should produce significant thrust")
        self.assertGreater(ab_thrust, 1000.0, "Afterburner should produce significant thrust")

        del fdm

    def test_turboprop_engine(self):
        """Test turboprop engine variant (C-130)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("C130")
        fdm.run_ic()

        # Verify turboprop-specific properties
        # Turboprops should have N2 (gas generator speed)
        n2 = fdm["propulsion/engine[0]/n2"]
        self.assertIsNotNone(n2, "Turboprop N2 should be accessible")

        # Turboprops produce thrust through propeller
        thrust = fdm["propulsion/engine[0]/thrust-lbs"]
        self.assertIsNotNone(thrust, "Turboprop thrust should be accessible")

        # Set engine running
        fdm["propulsion/engine[0]/set-running"] = 1

        # Set throttle and run
        fdm["fcs/throttle-cmd-norm[0]"] = 0.6
        for _ in range(50):
            fdm.run()

        # Check that engine responds to throttle
        thrust_power = fdm["propulsion/engine[0]/thrust-lbs"]
        self.assertGreater(thrust_power, 0.0, "Turboprop should produce positive thrust")

        del fdm

    def test_turbine_multiple_engines(self):
        """Test multiple turbine engines operating together."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")

        fdm["ic/h-sl-ft"] = 10000.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        # Set both engines running
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["propulsion/engine[1]/set-running"] = 1

        # Set different throttle on each engine
        fdm["fcs/throttle-cmd-norm[0]"] = 0.6
        fdm["fcs/throttle-cmd-norm[1]"] = 0.8

        for _ in range(50):
            fdm.run()

        # Verify both engines are producing thrust
        thrust_0 = fdm["propulsion/engine[0]/thrust-lbs"]
        thrust_1 = fdm["propulsion/engine[1]/thrust-lbs"]

        self.assertGreater(thrust_0, 0.0, "Engine 0 should produce thrust")
        self.assertGreater(thrust_1, 0.0, "Engine 1 should produce thrust")

        # Engine 1 should produce more thrust (higher throttle)
        self.assertGreater(thrust_1, thrust_0, "Higher throttle should produce more thrust")

        del fdm

    def test_turbine_startup_properties(self):
        """Test turbine engine properties at startup."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("737")
        fdm.run_ic()

        # At startup with zero throttle, check initial conditions
        initial_n1 = fdm["propulsion/engine[0]/n1"]
        initial_n2 = fdm["propulsion/engine[0]/n2"]
        initial_thrust = fdm["propulsion/engine[0]/thrust-lbs"]

        # Run simulation to stabilize initial state
        for _ in range(20):
            fdm.run()

        # Verify properties are accessible after running
        n1_after = fdm["propulsion/engine[0]/n1"]
        n2_after = fdm["propulsion/engine[0]/n2"]

        self.assertIsNotNone(initial_n1)
        self.assertIsNotNone(initial_n2)
        self.assertIsNotNone(initial_thrust)
        self.assertIsNotNone(n1_after)
        self.assertIsNotNone(n2_after)

        del fdm


RunTest(TestTurbineBasic)
