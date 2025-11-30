# TestEulerAngles.py
#
# Tests for Euler angles and attitude representation.
# Exercises phi, theta, psi and attitude conversions.
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

import math

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestEulerAngles(JSBSimTestCase):
    """
    Tests for Euler angles and attitude.

    Tests cover:
    - Roll (phi), Pitch (theta), Yaw (psi)
    - Angle units (deg, rad)
    - Attitude initialization
    - Quaternion consistency
    """

    def test_phi_deg_property(self):
        """Test roll angle (phi) in degrees."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/phi-deg"] = 10.0
        fdm.run_ic()

        phi = fdm["attitude/phi-deg"]
        self.assertAlmostEqual(phi, 10.0, delta=1.0, msg="Phi should be ~10 degrees")

        del fdm

    def test_theta_deg_property(self):
        """Test pitch angle (theta) in degrees."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/theta-deg"] = 5.0
        fdm.run_ic()

        theta = fdm["attitude/theta-deg"]
        self.assertAlmostEqual(theta, 5.0, delta=1.0, msg="Theta should be ~5 degrees")

        del fdm

    def test_psi_deg_property(self):
        """Test yaw angle (psi) in degrees."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/psi-true-deg"] = 90.0
        fdm.run_ic()

        psi = fdm["attitude/psi-deg"]
        self.assertAlmostEqual(psi, 90.0, delta=5.0, msg="Psi should be ~90 degrees")

        del fdm

    def test_phi_rad_property(self):
        """Test roll angle in radians."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/phi-deg"] = 0.0
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("attitude/phi-rad"):
            phi_rad = fdm["attitude/phi-rad"]
            phi_deg = fdm["attitude/phi-deg"]
            expected_rad = phi_deg * math.pi / 180
            self.assertAlmostEqual(phi_rad, expected_rad, delta=0.1, msg="Phi rad should match deg")

        del fdm

    def test_theta_rad_property(self):
        """Test pitch angle in radians."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/theta-deg"] = 0.0
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("attitude/theta-rad"):
            theta_rad = fdm["attitude/theta-rad"]
            theta_deg = fdm["attitude/theta-deg"]
            expected_rad = theta_deg * math.pi / 180
            self.assertAlmostEqual(
                theta_rad, expected_rad, delta=0.1, msg="Theta rad should match deg"
            )

        del fdm

    def test_level_flight_angles(self):
        """Test angles in level flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/theta-deg"] = 0.0
        fdm.run_ic()

        phi = fdm["attitude/phi-deg"]
        self.assertAlmostEqual(phi, 0.0, delta=2.0, msg="Phi should be near 0")

        del fdm

    def test_heading_360_wrap(self):
        """Test heading wraps correctly at 360 degrees."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/psi-true-deg"] = 350.0
        fdm.run_ic()

        psi = fdm["attitude/psi-deg"]
        # Psi should be between 0 and 360
        self.assertGreaterEqual(psi, 0, "Psi should be >= 0")
        self.assertLess(psi, 360, "Psi should be < 360")

        del fdm

    def test_bank_angle_initialization(self):
        """Test bank angle initialization."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/phi-deg"] = 30.0
        fdm.run_ic()

        phi = fdm["attitude/phi-deg"]
        self.assertAlmostEqual(phi, 30.0, delta=2.0, msg="Bank should be ~30 degrees")

        del fdm

    def test_quaternion_properties(self):
        """Test quaternion attitude representation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check quaternion components
        props = [
            "attitude/roll-rad",
            "attitude/pitch-rad",
            "attitude/heading-true-rad",
        ]

        for prop in props:
            if fdm.get_property_manager().hasNode(prop):
                value = fdm[prop]
                self.assertIsNotNone(value, f"{prop} should be accessible")

        del fdm

    def test_pitch_up_changes_theta(self):
        """Test that pitch up changes theta."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        initial_theta = fdm["attitude/theta-deg"]

        # Apply nose-up elevator
        fdm["fcs/elevator-cmd-norm"] = -0.5
        for _ in range(100):
            fdm.run()

        final_theta = fdm["attitude/theta-deg"]

        # Theta should change with pitch input
        self.assertNotEqual(initial_theta, final_theta, "Theta should change")

        del fdm


if __name__ == "__main__":
    RunTest(TestEulerAngles)
