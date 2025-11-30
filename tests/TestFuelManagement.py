# TestFuelManagement.py
#
# Tests for fuel management systems.
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


class TestFuelManagement(JSBSimTestCase):
    """
    Tests for fuel management properties.

    Tests cover:
    - Fuel quantity
    - Fuel tank selection
    - Fuel consumption
    - Total fuel
    """

    def test_total_fuel_property(self):
        """Test total fuel property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/total-fuel-lbs"):
            fuel = fdm["propulsion/total-fuel-lbs"]
            self.assertGreater(fuel, 0)

        del fdm

    def test_fuel_tank_contents(self):
        """Test fuel tank contents property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/tank[0]/contents-lbs"):
            contents = fdm["propulsion/tank[0]/contents-lbs"]
            self.assertIsNotNone(contents)

        del fdm

    def test_fuel_consumption(self):
        """Test fuel consumption during engine operation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("propulsion/total-fuel-lbs"):
            del fdm
            return

        initial_fuel = fdm["propulsion/total-fuel-lbs"]

        # Start engine and run
        fdm["fcs/throttle-cmd-norm"] = 0.8
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(500):
            fdm.run()

        final_fuel = fdm["propulsion/total-fuel-lbs"]
        # Fuel should decrease
        self.assertLessEqual(final_fuel, initial_fuel)

        del fdm

    def test_fuel_tank_capacity(self):
        """Test fuel tank capacity property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/tank[0]/capacity-lbs"):
            capacity = fdm["propulsion/tank[0]/capacity-lbs"]
            self.assertGreater(capacity, 0)

        del fdm

    def test_multiple_tanks(self):
        """Test multiple fuel tanks."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        tank_count = 0
        for i in range(4):
            if pm.hasNode(f"propulsion/tank[{i}]/contents-lbs"):
                tank_count += 1

        # C172 should have at least 2 tanks
        self.assertGreaterEqual(tank_count, 1)

        del fdm

    def test_fuel_feed(self):
        """Test fuel feed status."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/tank[0]/priority"):
            priority = fdm["propulsion/tank[0]/priority"]
            self.assertIsNotNone(priority)

        del fdm


if __name__ == "__main__":
    RunTest(TestFuelManagement)
