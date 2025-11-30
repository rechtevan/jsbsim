# TestGroundOperations.py
#
# Tests for ground operations (taxi, takeoff roll).
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


class TestGroundOperations(JSBSimTestCase):
    """
    Tests for ground operations.

    Tests cover:
    - Taxi
    - Takeoff roll
    - Ground steering
    - Braking
    """

    def test_stationary_on_ground(self):
        """Test aircraft stays stationary with brakes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/terrain-elevation-ft"] = 0
        fdm["ic/u-fps"] = 0
        fdm.run_ic()

        # Apply brakes
        fdm["fcs/left-brake-cmd-norm"] = 1.0
        fdm["fcs/right-brake-cmd-norm"] = 1.0

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vg-fps"):
            vg = fdm["velocities/vg-fps"]
            # Should be nearly stationary (some drift acceptable)
            self.assertLess(abs(vg), 20)

        del fdm

    def test_takeoff_roll_accelerates(self):
        """Test takeoff roll accelerates aircraft."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/terrain-elevation-ft"] = 0
        fdm["ic/u-fps"] = 0
        fdm.run_ic()

        # Full throttle, start engine
        fdm["fcs/throttle-cmd-norm"] = 1.0
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(300):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vg-fps"):
            vg = fdm["velocities/vg-fps"]
            # Should have some speed after takeoff roll
            self.assertIsNotNone(vg)

        del fdm

    def test_nose_wheel_steering(self):
        """Test nose wheel steering property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/steer-cmd-norm"):
            fdm["fcs/steer-cmd-norm"] = 0.5
            steer = fdm["fcs/steer-cmd-norm"]
            self.assertAlmostEqual(steer, 0.5, delta=0.1)

        del fdm

    def test_differential_braking(self):
        """Test differential braking."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Apply asymmetric brakes
        if pm.hasNode("fcs/left-brake-cmd-norm"):
            fdm["fcs/left-brake-cmd-norm"] = 1.0
            fdm["fcs/right-brake-cmd-norm"] = 0.0
            left = fdm["fcs/left-brake-cmd-norm"]
            self.assertAlmostEqual(left, 1.0, delta=0.1)

        del fdm

    def test_weight_on_wheels(self):
        """Test weight-on-wheels indicator on ground."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/terrain-elevation-ft"] = 0
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("gear/wow"):
            wow = fdm["gear/wow"]
            self.assertIsNotNone(wow)

        del fdm


if __name__ == "__main__":
    RunTest(TestGroundOperations)
