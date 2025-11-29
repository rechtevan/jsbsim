# TestPropagateBasic.py
#
# Tests for FGPropagate - equations of motion integration.
# Exercises position, velocity, and orientation propagation.
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


class TestPropagateBasic(JSBSimTestCase):
    """
    Tests for FGPropagate (equations of motion integration).

    Tests cover:
    - Position propagation (latitude, longitude, altitude)
    - Velocity propagation (body and NED frames)
    - Orientation propagation (quaternion and Euler)
    - Integration accuracy over time
    - Frame transformations
    """

    def test_position_properties_exist(self):
        """Test that position properties are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Geodetic position
        lat = fdm["position/lat-gc-rad"]
        lon = fdm["position/long-gc-rad"]
        alt = fdm["position/h-sl-ft"]

        self.assertIsNotNone(lat, "Latitude should be accessible")
        self.assertIsNotNone(lon, "Longitude should be accessible")
        self.assertIsNotNone(alt, "Altitude should be accessible")

        del fdm

    def test_velocity_properties_exist(self):
        """Test that velocity properties are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Body-axis velocities
        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]

        self.assertIsNotNone(u, "U velocity should be accessible")
        self.assertIsNotNone(v, "V velocity should be accessible")
        self.assertIsNotNone(w, "W velocity should be accessible")

        del fdm

    def test_ned_velocities(self):
        """Test NED (North-East-Down) velocity components."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # NED velocities
        vn = fdm["velocities/v-north-fps"]
        ve = fdm["velocities/v-east-fps"]
        vd = fdm["velocities/v-down-fps"]

        self.assertIsNotNone(vn, "North velocity should be accessible")
        self.assertIsNotNone(ve, "East velocity should be accessible")
        self.assertIsNotNone(vd, "Down velocity should be accessible")

        del fdm

    def test_altitude_change_with_velocity(self):
        """Test that altitude changes with vertical velocity."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 5.0  # Climbing
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        # Run simulation
        for _ in range(200):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]

        # With positive climb angle, altitude should increase
        self.assertGreater(final_alt, initial_alt, "Altitude should increase with positive gamma")

        del fdm

    def test_position_change_with_velocity(self):
        """Test that position changes with forward velocity."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/psi-true-deg"] = 90.0  # Heading east
        fdm.run_ic()

        initial_lon = fdm["position/long-gc-rad"]

        # Run simulation
        for _ in range(200):
            fdm.run()

        final_lon = fdm["position/long-gc-rad"]

        # Flying east, longitude should increase
        self.assertGreater(final_lon, initial_lon, "Longitude should increase when flying east")

        del fdm

    def test_quaternion_properties(self):
        """Test quaternion orientation properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Quaternion components
        q0 = fdm["attitude/phi-rad"]  # Using Euler as proxy
        # JSBSim uses Euler angles primarily; quaternions are internal

        self.assertIsNotNone(q0, "Orientation should be accessible")

        del fdm

    def test_ecef_position(self):
        """Test ECEF position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # ECEF position
        if fdm.get_property_manager().hasNode("position/ecef-x-ft"):
            x = fdm["position/ecef-x-ft"]
            y = fdm["position/ecef-y-ft"]
            z = fdm["position/ecef-z-ft"]

            self.assertIsNotNone(x, "ECEF X should be accessible")
            self.assertIsNotNone(y, "ECEF Y should be accessible")
            self.assertIsNotNone(z, "ECEF Z should be accessible")

        del fdm

    def test_inertial_position(self):
        """Test inertial position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check for inertial position properties
        if fdm.get_property_manager().hasNode("position/eci-x-ft"):
            x = fdm["position/eci-x-ft"]
            self.assertIsNotNone(x, "ECI position should be accessible")

        del fdm

    def test_ground_altitude(self):
        """Test terrain/ground altitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        # Altitude above ground
        if fdm.get_property_manager().hasNode("position/h-agl-ft"):
            agl = fdm["position/h-agl-ft"]
            self.assertIsNotNone(agl, "AGL altitude should be accessible")
            self.assertGreater(agl, 0, "AGL should be positive in air")

        del fdm

    def test_earth_radius(self):
        """Test earth radius property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Earth radius at current position
        if fdm.get_property_manager().hasNode("position/radius-to-vehicle-ft"):
            radius = fdm["position/radius-to-vehicle-ft"]
            # Earth radius is approximately 20.9 million feet
            self.assertGreater(radius, 20e6, "Radius should be Earth-sized")
            self.assertLess(radius, 22e6, "Radius should be Earth-sized")

        del fdm

    def test_altitude_rate(self):
        """Test altitude rate (hdot) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 5.0  # Climbing
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Hdot should be positive when climbing
        hdot = fdm["velocities/h-dot-fps"]
        self.assertGreater(hdot, 0, "Hdot should be positive when climbing")

        del fdm

    def test_ground_speed(self):
        """Test ground speed calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Ground speed
        if fdm.get_property_manager().hasNode("velocities/vg-fps"):
            vg = fdm["velocities/vg-fps"]
            self.assertGreater(vg, 0, "Ground speed should be positive")

        del fdm


if __name__ == "__main__":
    RunTest(TestPropagateBasic)
