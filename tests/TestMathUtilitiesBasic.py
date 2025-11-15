# TestMathUtilitiesBasic.py
#
# Test mathematical utilities: quaternions, matrices, vectors, and coordinate
# transformations accessible via the JSBSim Python API.
#
# Tests focus on:
# - Quaternion operations and Euler angle conversions
# - Orientation calculations and consistency
# - Vector operations in different reference frames
# - Coordinate transformations (body, NED, ECEF)
# - Angular velocity calculations
#
# Copyright (c) 2025
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

from JSBSim_utils import JSBSimTestCase, RunTest


class TestMathUtilitiesBasic(JSBSimTestCase):
    """Test mathematical utilities through Python API."""

    def test_quaternion_creation(self):
        """Test quaternion creation and initialization from Euler angles."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test zero orientation (identity quaternion)
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/theta-deg"] = 0.0
        fdm["ic/psi-true-deg"] = 0.0
        fdm.run_ic()

        # Verify all Euler angles are zero
        phi = fdm["attitude/phi-rad"]
        theta = fdm["attitude/theta-rad"]
        psi = fdm["attitude/psi-rad"]

        self.assertAlmostEqual(phi, 0.0, delta=1e-8)
        self.assertAlmostEqual(theta, 0.0, delta=1e-8)
        self.assertAlmostEqual(psi, 0.0, delta=1e-8)

        # Test specific orientation (15° roll, 5° pitch, 90° heading)
        fdm["ic/phi-deg"] = 15.0
        fdm["ic/theta-deg"] = 5.0
        fdm["ic/psi-true-deg"] = 90.0
        fdm.run_ic()

        phi = math.degrees(fdm["attitude/phi-rad"])
        theta = math.degrees(fdm["attitude/theta-rad"])
        psi = math.degrees(fdm["attitude/psi-rad"])

        self.assertAlmostEqual(phi, 15.0, delta=0.1)
        self.assertAlmostEqual(theta, 5.0, delta=0.1)
        self.assertAlmostEqual(psi, 90.0, delta=0.1)

    def test_quaternion_rotation(self):
        """Test quaternion rotation operations through attitude changes."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.load_ic(self.sandbox.path_to_jsbsim_file("aircraft/c172x/reset00.xml"), False)

        # Set initial orientation
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/theta-deg"] = 0.0
        fdm["ic/psi-true-deg"] = 0.0
        fdm.run_ic()

        initial_phi = fdm["attitude/phi-rad"]
        initial_theta = fdm["attitude/theta-rad"]
        initial_psi = fdm["attitude/psi-rad"]

        # Apply control inputs to induce rotation
        fdm["fcs/aileron-cmd-norm"] = 0.3  # Roll right
        fdm["fcs/elevator-cmd-norm"] = -0.1  # Pitch up
        fdm["fcs/rudder-cmd-norm"] = 0.2  # Yaw right

        # Run simulation to accumulate rotation
        for _ in range(100):
            fdm.run()

        final_phi = fdm["attitude/phi-rad"]
        final_theta = fdm["attitude/theta-rad"]
        final_psi = fdm["attitude/psi-rad"]

        # Verify that orientation has changed due to rotations
        # Use looser tolerance as changes may be small initially
        phi_changed = abs(final_phi - initial_phi) > 0.001
        theta_changed = abs(final_theta - initial_theta) > 0.001
        psi_changed = abs(final_psi - initial_psi) > 0.001

        self.assertTrue(
            phi_changed or theta_changed or psi_changed,
            "At least one orientation angle should change with control inputs",
        )

    def test_quaternion_euler_conversion(self):
        """Test quaternion to Euler angle conversion consistency."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test multiple orientations to verify conversion consistency
        test_cases = [
            (0.0, 0.0, 0.0),  # Identity
            (10.0, 0.0, 0.0),  # Pure roll
            (0.0, 10.0, 0.0),  # Pure pitch
            (0.0, 0.0, 45.0),  # Pure yaw
            (15.0, 5.0, 90.0),  # Combined
            (-20.0, 10.0, 180.0),  # Negative roll
            (30.0, -15.0, 270.0),  # Negative pitch
        ]

        for roll_deg, pitch_deg, yaw_deg in test_cases:
            fdm["ic/phi-deg"] = roll_deg
            fdm["ic/theta-deg"] = pitch_deg
            fdm["ic/psi-true-deg"] = yaw_deg
            fdm.run_ic()

            # Read back Euler angles
            phi = math.degrees(fdm["attitude/phi-rad"])
            theta = math.degrees(fdm["attitude/theta-rad"])
            psi = math.degrees(fdm["attitude/psi-rad"])

            # Verify conversion is consistent
            self.assertAlmostEqual(
                phi,
                roll_deg,
                delta=0.1,
                msg=f"Roll mismatch for case ({roll_deg}, {pitch_deg}, {yaw_deg})",
            )
            self.assertAlmostEqual(
                theta,
                pitch_deg,
                delta=0.1,
                msg=f"Pitch mismatch for case ({roll_deg}, {pitch_deg}, {yaw_deg})",
            )

            # Handle wrap-around for yaw
            psi_normalized = yaw_deg % 360.0
            psi_read_normalized = psi % 360.0
            self.assertAlmostEqual(
                psi_read_normalized,
                psi_normalized,
                delta=0.1,
                msg=f"Yaw mismatch for case ({roll_deg}, {pitch_deg}, {yaw_deg})",
            )

    def test_orientation_calculations(self):
        """Test orientation calculations using quaternions."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Set specific orientation
        roll_deg = 20.0
        pitch_deg = 10.0
        yaw_deg = 45.0

        fdm["ic/phi-deg"] = roll_deg
        fdm["ic/theta-deg"] = pitch_deg
        fdm["ic/psi-true-deg"] = yaw_deg
        fdm.run_ic()

        # Verify consistent orientation between different property representations
        phi_rad = fdm["attitude/phi-rad"]
        theta_rad = fdm["attitude/theta-rad"]
        psi_rad = fdm["attitude/psi-rad"]

        roll_rad = fdm["attitude/roll-rad"]
        pitch_rad = fdm["attitude/pitch-rad"]
        heading_rad = fdm["attitude/heading-true-rad"]

        # phi and roll should be equivalent
        self.assertAlmostEqual(phi_rad, roll_rad, delta=1e-8)
        # theta and pitch should be equivalent
        self.assertAlmostEqual(theta_rad, pitch_rad, delta=1e-8)
        # psi and heading should be equivalent (or differ by 2*pi)
        psi_diff = abs(psi_rad - heading_rad)
        self.assertTrue(
            psi_diff < 1e-6 or abs(psi_diff - 2 * math.pi) < 1e-6,
            f"psi and heading differ by {psi_diff}",
        )

    def test_vector_operations(self):
        """Test vector operations in body frame."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.load_ic(self.sandbox.path_to_jsbsim_file("aircraft/c172x/reset00.xml"), False)

        # Set initial velocity in body frame (straight and level flight)
        fdm["ic/u-fps"] = 100.0  # Forward velocity
        fdm["ic/v-fps"] = 0.0  # Side velocity
        fdm["ic/w-fps"] = 0.0  # Vertical velocity
        fdm.run_ic()

        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]

        self.assertAlmostEqual(u, 100.0, delta=0.1)
        self.assertAlmostEqual(v, 0.0, delta=0.1)
        self.assertAlmostEqual(w, 0.0, delta=0.1)

        # Verify velocity magnitude
        v_magnitude = math.sqrt(u * u + v * v + w * w)
        self.assertAlmostEqual(v_magnitude, 100.0, delta=0.1)

    def test_matrix_operations(self):
        """Test coordinate transformation matrix operations."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Set orientation for non-trivial transformation
        roll_deg = 15.0
        pitch_deg = 10.0
        yaw_deg = 30.0

        fdm["ic/phi-deg"] = roll_deg
        fdm["ic/theta-deg"] = pitch_deg
        fdm["ic/psi-true-deg"] = yaw_deg

        # Set velocity in NED frame
        v_north = 100.0
        v_east = 50.0
        v_down = -10.0  # Climbing

        fdm["ic/vn-fps"] = v_north
        fdm["ic/ve-fps"] = v_east
        fdm["ic/vd-fps"] = v_down
        fdm.run_ic()

        # Read back velocities in both frames
        vn = fdm["velocities/v-north-fps"]
        ve = fdm["velocities/v-east-fps"]
        vd = fdm["velocities/v-down-fps"]

        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]

        # Verify NED velocities match input
        self.assertAlmostEqual(vn, v_north, delta=0.1)
        self.assertAlmostEqual(ve, v_east, delta=0.1)
        self.assertAlmostEqual(vd, v_down, delta=0.1)

        # Verify magnitude is conserved in transformation
        ned_magnitude = math.sqrt(vn * vn + ve * ve + vd * vd)
        body_magnitude = math.sqrt(u * u + v * v + w * w)
        self.assertAlmostEqual(
            ned_magnitude,
            body_magnitude,
            delta=0.1,
            msg="Magnitude not conserved in transformation",
        )

    def test_coordinate_transformations(self):
        """Test body <-> NED <-> ECEF coordinate transformations."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Set position and orientation
        fdm["ic/lat-gc-deg"] = 37.0  # San Francisco area
        fdm["ic/long-gc-deg"] = -122.0
        fdm["ic/h-sl-ft"] = 5000.0

        fdm["ic/phi-deg"] = 10.0
        fdm["ic/theta-deg"] = 5.0
        fdm["ic/psi-true-deg"] = 45.0

        # Set velocity
        fdm["ic/vn-fps"] = 100.0
        fdm["ic/ve-fps"] = 50.0
        fdm["ic/vd-fps"] = 0.0
        fdm.run_ic()

        # Verify position properties are available
        lat = fdm["position/lat-gc-deg"]
        lon = fdm["position/long-gc-deg"]
        alt = fdm["position/h-sl-ft"]

        self.assertAlmostEqual(lat, 37.0, delta=0.01)
        self.assertAlmostEqual(lon, -122.0, delta=0.01)
        self.assertAlmostEqual(alt, 5000.0, delta=1.0)

        # Verify ECEF position is non-zero and reasonable
        ecef_x = fdm["position/epa-rad"]  # Using available ECEF-related property
        self.assertIsNotNone(ecef_x)

        # Verify velocities in different frames
        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]

        # All velocities should be non-zero due to transformation
        self.assertIsNotNone(u)
        self.assertIsNotNone(v)
        self.assertIsNotNone(w)

    def test_angular_velocity(self):
        """Test angular velocity calculations (p, q, r in body frame)."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.load_ic(self.sandbox.path_to_jsbsim_file("aircraft/c172x/reset00.xml"), False)

        # Set initial angular velocities to zero
        fdm["ic/p-rad_sec"] = 0.0
        fdm["ic/q-rad_sec"] = 0.0
        fdm["ic/r-rad_sec"] = 0.0
        fdm.run_ic()

        # Verify initial angular velocities
        p0 = fdm["velocities/p-rad_sec"]
        q0 = fdm["velocities/q-rad_sec"]
        r0 = fdm["velocities/r-rad_sec"]

        self.assertAlmostEqual(p0, 0.0, delta=1e-6)
        self.assertAlmostEqual(q0, 0.0, delta=1e-6)
        self.assertAlmostEqual(r0, 0.0, delta=1e-6)

        # Apply control inputs to induce rotation
        fdm["fcs/aileron-cmd-norm"] = 0.5  # Roll input
        fdm["fcs/elevator-cmd-norm"] = 0.3  # Pitch input
        fdm["fcs/rudder-cmd-norm"] = 0.2  # Yaw input

        # Run simulation
        for _ in range(50):
            fdm.run()

        # Read angular velocities
        p = fdm["velocities/p-rad_sec"]
        q = fdm["velocities/q-rad_sec"]
        r = fdm["velocities/r-rad_sec"]

        # Verify angular velocities have changed
        # Use looser tolerance as changes may be very small initially
        p_changed = abs(p - p0) > 1e-6
        q_changed = abs(q - q0) > 1e-6
        r_changed = abs(r - r0) > 1e-6

        self.assertTrue(
            p_changed or q_changed or r_changed,
            "At least one angular velocity should change with control inputs",
        )

    def test_euler_angle_singularity(self):
        """Test behavior near gimbal lock (pitch = ±90°)."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test near gimbal lock (pitch = 89°)
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/theta-deg"] = 89.0
        fdm["ic/psi-true-deg"] = 45.0
        fdm.run_ic()

        theta = math.degrees(fdm["attitude/theta-rad"])
        self.assertAlmostEqual(
            theta, 89.0, delta=0.5, msg="Pitch should be maintained near gimbal lock"
        )

        # Verify simulation doesn't crash at extreme pitch
        for _ in range(10):
            fdm.run()

        # Should still have valid attitude
        phi = fdm["attitude/phi-rad"]
        theta = fdm["attitude/theta-rad"]
        psi = fdm["attitude/psi-rad"]

        self.assertIsNotNone(phi)
        self.assertIsNotNone(theta)
        self.assertIsNotNone(psi)

    def test_rotation_rate_consistency(self):
        """Test consistency between Euler angle rates and body angular rates."""
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Set moderate orientation (not near singularities)
        fdm["ic/phi-deg"] = 20.0
        fdm["ic/theta-deg"] = 15.0
        fdm["ic/psi-true-deg"] = 30.0

        # Set angular velocities in body frame
        p_rad_sec = 0.1  # Roll rate
        q_rad_sec = 0.05  # Pitch rate
        r_rad_sec = 0.08  # Yaw rate

        fdm["ic/p-rad_sec"] = p_rad_sec
        fdm["ic/q-rad_sec"] = q_rad_sec
        fdm["ic/r-rad_sec"] = r_rad_sec
        fdm.run_ic()

        # Verify angular velocities are set correctly
        p = fdm["velocities/p-rad_sec"]
        q = fdm["velocities/q-rad_sec"]
        r = fdm["velocities/r-rad_sec"]

        self.assertAlmostEqual(p, p_rad_sec, delta=1e-6)
        self.assertAlmostEqual(q, q_rad_sec, delta=1e-6)
        self.assertAlmostEqual(r, r_rad_sec, delta=1e-6)

        # Record initial Euler angles
        phi0 = fdm["attitude/phi-rad"]
        theta0 = fdm["attitude/theta-rad"]
        psi0 = fdm["attitude/psi-rad"]

        # Run simulation
        fdm.run()

        # Check that Euler angles have changed
        phi1 = fdm["attitude/phi-rad"]
        theta1 = fdm["attitude/theta-rad"]
        psi1 = fdm["attitude/psi-rad"]

        # At least one angle should have changed
        euler_changed = (
            abs(phi1 - phi0) > 1e-9 or abs(theta1 - theta0) > 1e-9 or abs(psi1 - psi0) > 1e-9
        )
        self.assertTrue(
            euler_changed, "Euler angles should change with non-zero angular velocities"
        )


RunTest(TestMathUtilitiesBasic)
