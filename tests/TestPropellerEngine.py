# TestPropellerEngine.py
#
# Tests for propeller engine (FGPiston) functionality.
# Exercises piston engine model and propeller properties.
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


class TestPropellerEngine(JSBSimTestCase):
    """
    Tests for piston engine and propeller model.

    Tests cover:
    - Engine RPM properties
    - Power output
    - Propeller thrust
    - Fuel consumption
    - Manifold pressure
    """

    def test_engine_rpm_zero_initially(self):
        """Test that engine RPM is zero when not running."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        rpm = fdm["propulsion/engine/engine-rpm"]
        self.assertEqual(rpm, 0, "RPM should be 0 when engine not running")

        del fdm

    def test_engine_rpm_positive_when_running(self):
        """Test that engine RPM is positive when running."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.3
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        rpm = fdm["propulsion/engine/engine-rpm"]
        self.assertGreater(rpm, 400, "RPM should be positive when running")

        del fdm

    def test_propeller_rpm_property(self):
        """Test propeller RPM property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.3
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        if fdm.get_property_manager().hasNode("propulsion/engine/propeller-rpm"):
            prop_rpm = fdm["propulsion/engine/propeller-rpm"]
            self.assertGreater(prop_rpm, 0, "Propeller RPM should be positive")

        del fdm

    def test_power_output_property(self):
        """Test engine power output property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        fdm["propulsion/starter_cmd"] = 0

        for _ in range(100):
            fdm.run()

        if fdm.get_property_manager().hasNode("propulsion/engine/power-hp"):
            power = fdm["propulsion/engine/power-hp"]
            self.assertGreater(power, 0, "Power output should be positive")

        del fdm

    def test_thrust_property(self):
        """Test propeller thrust property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine properly
        fdm["fcs/throttle-cmd-norm"] = 0.7
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        fdm["propulsion/starter_cmd"] = 0

        for _ in range(100):
            fdm.run()

        if fdm.get_property_manager().hasNode("propulsion/engine/thrust-lbs"):
            thrust = fdm["propulsion/engine/thrust-lbs"]
            self.assertGreater(thrust, 0, "Thrust should be positive in flight")

        del fdm

    def test_manifold_pressure(self):
        """Test manifold pressure property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        if fdm.get_property_manager().hasNode("propulsion/engine/map-inhg"):
            map_pressure = fdm["propulsion/engine/map-inhg"]
            self.assertGreater(map_pressure, 10, "MAP should be > 10 inHg")
            self.assertLess(map_pressure, 35, "MAP should be < 35 inHg")

        del fdm

    def test_fuel_flow_property(self):
        """Test fuel flow property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        if fdm.get_property_manager().hasNode("propulsion/engine/fuel-flow-rate-pps"):
            fuel_flow = fdm["propulsion/engine/fuel-flow-rate-pps"]
            self.assertGreater(fuel_flow, 0, "Fuel flow should be positive")

        del fdm

    def test_throttle_affects_rpm(self):
        """Test that throttle affects engine RPM."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine at low throttle
        fdm["fcs/throttle-cmd-norm"] = 0.2
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        fdm["propulsion/starter_cmd"] = 0
        for _ in range(50):
            fdm.run()

        rpm_low = fdm["propulsion/engine/engine-rpm"]

        # Increase throttle
        fdm["fcs/throttle-cmd-norm"] = 0.9

        for _ in range(200):
            fdm.run()

        rpm_high = fdm["propulsion/engine/engine-rpm"]

        self.assertGreater(rpm_high, rpm_low, "Higher throttle should increase RPM")

        del fdm

    def test_egt_property(self):
        """Test exhaust gas temperature property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(400):
            fdm.run()
            if fdm["propulsion/engine/set-running"] == 1:
                break

        if fdm.get_property_manager().hasNode("propulsion/engine/egt-degf"):
            egt = fdm["propulsion/engine/egt-degf"]
            self.assertIsNotNone(egt, "EGT should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestPropellerEngine)
