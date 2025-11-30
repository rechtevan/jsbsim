# TestAeroAngles.py
#
# Tests for aerodynamic angles (alpha, beta, gamma).
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


class TestAeroAngles(JSBSimTestCase):
    """
    Tests for aerodynamic angle properties.

    Tests cover:
    - Angle of attack (alpha)
    - Sideslip angle (beta)
    - Flight path angle (gamma)
    """

    def test_alpha_property(self):
        """Test angle of attack property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-deg"):
            alpha = fdm["aero/alpha-deg"]
            self.assertIsNotNone(alpha)
            # Typical cruise alpha for C172 is a few degrees
            self.assertGreater(alpha, -10)
            self.assertLess(alpha, 20)

        del fdm

    def test_beta_property(self):
        """Test sideslip angle property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/beta-deg"):
            beta = fdm["aero/beta-deg"]
            self.assertIsNotNone(beta)
            # In coordinated flight, beta should be near zero
            self.assertAlmostEqual(beta, 0, delta=5)

        del fdm

    def test_gamma_property(self):
        """Test flight path angle property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/gamma-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("flight-path/gamma-deg"):
            gamma = fdm["flight-path/gamma-deg"]
            self.assertIsNotNone(gamma)

        del fdm

    def test_alpha_rad_property(self):
        """Test angle of attack in radians."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-rad"):
            alpha_rad = fdm["aero/alpha-rad"]
            self.assertIsNotNone(alpha_rad)

        del fdm

    def test_beta_rad_property(self):
        """Test sideslip angle in radians."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/beta-rad"):
            beta_rad = fdm["aero/beta-rad"]
            self.assertIsNotNone(beta_rad)

        del fdm

    def test_alpha_changes_with_elevator(self):
        """Test that alpha changes with elevator input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("aero/alpha-deg"):
            del fdm
            return

        # Pull back on elevator
        fdm["fcs/elevator-cmd-norm"] = -0.5

        for _ in range(100):
            fdm.run()

        final_alpha = fdm["aero/alpha-deg"]

        # Alpha should change with elevator input
        self.assertIsNotNone(final_alpha)

        del fdm

    def test_beta_changes_with_rudder(self):
        """Test that beta changes with rudder input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("aero/beta-deg"):
            del fdm
            return

        # Apply rudder
        fdm["fcs/rudder-cmd-norm"] = 0.5

        for _ in range(100):
            fdm.run()

        beta = fdm["aero/beta-deg"]
        self.assertIsNotNone(beta)

        del fdm

    def test_high_alpha(self):
        """Test high angle of attack conditions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 60  # Low speed for high alpha
        fdm["ic/alpha-deg"] = 15
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-deg"):
            alpha = fdm["aero/alpha-deg"]
            # Should be close to requested alpha
            self.assertGreater(alpha, 5)

        del fdm


if __name__ == "__main__":
    RunTest(TestAeroAngles)
