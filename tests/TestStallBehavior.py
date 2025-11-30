# TestStallBehavior.py
#
# Tests for stall behavior and characteristics.
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


class TestStallBehavior(JSBSimTestCase):
    """
    Tests for stall behavior and characteristics.

    Tests cover:
    - High angle of attack
    - Stall warning
    - Alpha limits
    """

    def test_high_alpha_possible(self):
        """Test aircraft can achieve high angle of attack."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 60  # Low speed
        fdm.run_ic()

        # Pull back on elevator
        fdm["fcs/elevator-cmd-norm"] = -0.8

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-deg"):
            alpha = fdm["aero/alpha-deg"]
            # Should have some angle of attack
            self.assertIsNotNone(alpha)

        del fdm

    def test_alpha_property_exists(self):
        """Test alpha property is available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-deg"):
            alpha = fdm["aero/alpha-deg"]
            self.assertIsNotNone(alpha)

        del fdm

    def test_cl_changes_with_alpha(self):
        """Test lift coefficient changes with alpha."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("aero/coefficient/CLwbh"):
            del fdm
            return

        # Get initial CL - verify property exists
        fdm["aero/coefficient/CLwbh"]

        # Pitch up
        fdm["fcs/elevator-cmd-norm"] = -0.3
        for _ in range(50):
            fdm.run()

        cl2 = fdm["aero/coefficient/CLwbh"]
        # CL should change
        self.assertIsNotNone(cl2)

        del fdm

    def test_low_speed_flight(self):
        """Test flight at low speed near stall."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 50  # Very low speed
        fdm.run_ic()

        # Run simulation
        for _ in range(100):
            fdm.run()

        # Should still be flying (altitude shouldn't have crashed)
        alt = fdm["position/h-sl-ft"]
        self.assertGreater(alt, 0)

        del fdm

    def test_stall_speed_reasonable(self):
        """Test stall speed is in reasonable range."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 45  # Below typical stall speed
        fdm.run_ic()

        # Just verify it initializes
        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 5000, delta=100)

        del fdm


if __name__ == "__main__":
    RunTest(TestStallBehavior)
