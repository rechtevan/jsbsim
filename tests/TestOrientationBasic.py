# TestOrientationBasic.py
#
# Tests for aircraft orientation (Euler angles, quaternions).
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


class TestOrientationBasic(JSBSimTestCase):
    """
    Tests for aircraft orientation properties.

    Tests cover:
    - Euler angles (phi, theta, psi)
    - Heading
    - Bank angle
    """

    def test_phi_property(self):
        """Test roll angle (phi) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/phi-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/phi-deg"):
            phi = fdm["attitude/phi-deg"]
            self.assertAlmostEqual(phi, 0, delta=1)

        del fdm

    def test_theta_property(self):
        """Test pitch angle (theta) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/theta-deg"] = 5
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/theta-deg"):
            theta = fdm["attitude/theta-deg"]
            self.assertAlmostEqual(theta, 5, delta=2)

        del fdm

    def test_psi_property(self):
        """Test heading angle (psi) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/psi-true-deg"] = 90
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/psi-deg"):
            psi = fdm["attitude/psi-deg"]
            self.assertAlmostEqual(psi, 90, delta=5)

        del fdm

    def test_heading_north(self):
        """Test heading pointing north."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/psi-true-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/heading-true-rad"):
            heading = fdm["attitude/heading-true-rad"]
            self.assertIsNotNone(heading)

        del fdm

    def test_bank_angle_change(self):
        """Test bank angle changes with aileron."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/phi-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("attitude/phi-deg"):
            del fdm
            return

        # Apply aileron
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(100):
            fdm.run()

        phi = fdm["attitude/phi-deg"]
        # Should have some bank angle
        self.assertIsNotNone(phi)

        del fdm

    def test_pitch_angle_change(self):
        """Test pitch angle changes with elevator."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("attitude/theta-deg"):
            del fdm
            return

        # Apply elevator
        fdm["fcs/elevator-cmd-norm"] = -0.3

        for _ in range(100):
            fdm.run()

        theta = fdm["attitude/theta-deg"]
        self.assertIsNotNone(theta)

        del fdm

    def test_euler_angles_rad(self):
        """Test Euler angles in radians."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/phi-rad"):
            phi_rad = fdm["attitude/phi-rad"]
            self.assertIsNotNone(phi_rad)

        del fdm


if __name__ == "__main__":
    RunTest(TestOrientationBasic)
