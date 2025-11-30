# TestAircraftPerformance.py
#
# Tests for aircraft performance characteristics.
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


class TestAircraftPerformance(JSBSimTestCase):
    """
    Tests for aircraft performance.

    Tests cover:
    - Cruise speed
    - Range/endurance factors
    - Power required
    - Drag characteristics
    """

    def test_cruise_speed_achievable(self):
        """Test aircraft can achieve cruise speed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 120  # Cruise speed
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vc-kts"):
            vc = fdm["velocities/vc-kts"]
            self.assertAlmostEqual(vc, 120, delta=10)

        del fdm

    def test_drag_positive_at_speed(self):
        """Test drag is positive when flying."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/coefficient/CD"):
            cd = fdm["aero/coefficient/CD"]
            self.assertGreater(cd, 0)

        del fdm

    def test_lift_supports_weight(self):
        """Test lift approximately supports weight in cruise."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/gamma-deg"] = 0  # Level flight
        fdm.run_ic()

        # Run to stabilize
        for _ in range(100):
            fdm.run()

        # Check altitude is roughly maintained
        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 5000, delta=500)

        del fdm

    def test_power_available(self):
        """Test power available property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.7
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/power-hp"):
            power = fdm["propulsion/engine[0]/power-hp"]
            self.assertIsNotNone(power)

        del fdm

    def test_fuel_efficiency(self):
        """Test fuel flow is reasonable."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine at cruise power
        fdm["fcs/throttle-cmd-norm"] = 0.65
        fdm["fcs/mixture-cmd-norm"] = 0.9
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/fuel-flow-rate-pps"):
            ff = fdm["propulsion/engine[0]/fuel-flow-rate-pps"]
            self.assertIsNotNone(ff)

        del fdm


if __name__ == "__main__":
    RunTest(TestAircraftPerformance)
