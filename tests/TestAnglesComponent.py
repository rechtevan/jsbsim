# TestAnglesComponent.py
#
# Tests for angle-related calculations in JSBSim.
# Note: FGAngles component is rarely used in aircraft configurations,
# so we test angle properties and calculations available through
# standard aircraft models.
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


class TestAnglesComponent(JSBSimTestCase):
    """
    Tests for angle-related properties and calculations.

    Note: The FGAngles FCS component is available but rarely used
    in existing aircraft configurations. These tests verify
    angle-related properties through standard flight models.

    Tests cover:
    - Euler angle properties (phi, theta, psi)
    - Angle unit conversions
    - Heading calculations
    - Angular rate properties
    """

    def test_euler_angle_properties(self):
        """Test that Euler angle properties exist and are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Euler angles in radians
        phi = fdm["attitude/phi-rad"]
        theta = fdm["attitude/theta-rad"]
        psi = fdm["attitude/psi-rad"]

        self.assertIsNotNone(phi, "Roll angle should be accessible")
        self.assertIsNotNone(theta, "Pitch angle should be accessible")
        self.assertIsNotNone(psi, "Yaw angle should be accessible")

        del fdm

    def test_euler_angles_in_degrees(self):
        """Test Euler angles in degrees."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Euler angles in degrees
        phi_deg = fdm["attitude/phi-deg"]
        theta_deg = fdm["attitude/theta-deg"]
        psi_deg = fdm["attitude/psi-deg"]

        self.assertIsNotNone(phi_deg, "Roll angle in degrees should be accessible")
        self.assertIsNotNone(theta_deg, "Pitch angle in degrees should be accessible")
        self.assertIsNotNone(psi_deg, "Yaw angle in degrees should be accessible")

        del fdm

    def test_angle_unit_consistency(self):
        """Test that radian and degree properties are consistent."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/theta-deg"] = 5.0  # Some pitch angle
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        theta_rad = fdm["attitude/theta-rad"]
        theta_deg = fdm["attitude/theta-deg"]

        # Convert and compare
        expected_deg = theta_rad * (180.0 / math.pi)
        self.assertAlmostEqual(
            theta_deg, expected_deg, places=3, msg="Degree and radian should be consistent"
        )

        del fdm

    def test_heading_properties(self):
        """Test heading-related angle properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/psi-true-deg"] = 90.0  # East heading
        fdm.run_ic()

        # Heading properties
        heading_true = fdm["attitude/heading-true-rad"]

        self.assertIsNotNone(heading_true, "True heading should be accessible")

        del fdm

    def test_angular_rates(self):
        """Test angular rate properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Angular rates in rad/sec
        p = fdm["velocities/p-rad_sec"]
        q = fdm["velocities/q-rad_sec"]
        r = fdm["velocities/r-rad_sec"]

        self.assertIsNotNone(p, "Roll rate should be accessible")
        self.assertIsNotNone(q, "Pitch rate should be accessible")
        self.assertIsNotNone(r, "Yaw rate should be accessible")

        del fdm

    def test_alpha_beta_angles(self):
        """Test angle of attack and sideslip properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Aerodynamic angles
        alpha = fdm["aero/alpha-rad"]
        beta = fdm["aero/beta-rad"]

        self.assertIsNotNone(alpha, "Angle of attack should be accessible")
        self.assertIsNotNone(beta, "Sideslip angle should be accessible")

        del fdm

    def test_alpha_in_degrees(self):
        """Test angle of attack in degrees."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        alpha_rad = fdm["aero/alpha-rad"]
        alpha_deg = fdm["aero/alpha-deg"]

        # Should be consistent
        expected_deg = alpha_rad * (180.0 / math.pi)
        self.assertAlmostEqual(
            alpha_deg, expected_deg, places=3, msg="Alpha in degrees should be consistent"
        )

        del fdm

    def test_flight_path_angle(self):
        """Test flight path angle properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 0.0  # Level flight
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Flight path angle
        if fdm.get_property_manager().hasNode("flight-path/gamma-rad"):
            gamma = fdm["flight-path/gamma-rad"]
            self.assertIsNotNone(gamma, "Flight path angle should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestAnglesComponent)
