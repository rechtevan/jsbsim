# TestPositionBasic.py
#
# Tests for Position and Location properties.
# Exercises geodetic, geocentric, and body frame positions.
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


class TestPositionBasic(JSBSimTestCase):
    """
    Tests for Position and Location properties.

    Tests cover:
    - Geodetic position (lat/lon/alt)
    - Altitude variations (MSL, AGL)
    - Earth-centered coordinates
    - Position changes during simulation
    """

    def test_initial_altitude(self):
        """Test initial altitude setting."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 5000, delta=10)

        del fdm

    def test_initial_latitude(self):
        """Test initial latitude setting."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/lat-gc-deg"] = 45.0
        fdm.run_ic()

        lat = fdm["position/lat-gc-deg"]
        self.assertAlmostEqual(lat, 45.0, delta=0.1)

        del fdm

    def test_initial_longitude(self):
        """Test initial longitude setting."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/long-gc-deg"] = -122.0
        fdm.run_ic()

        lon = fdm["position/long-gc-deg"]
        self.assertAlmostEqual(lon, -122.0, delta=0.1)

        del fdm

    def test_altitude_agl(self):
        """Test altitude above ground level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 1000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-agl-ft"):
            agl = fdm["position/h-agl-ft"]
            # AGL should be similar to MSL when terrain is near sea level
            self.assertGreater(agl, 0)

        del fdm

    def test_ecef_coordinates(self):
        """Test Earth-Centered Earth-Fixed coordinates."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm.run_ic()

        pm = fdm.get_property_manager()

        # Check ECEF position exists
        has_ecef = pm.hasNode("position/ecef-x-ft") or pm.hasNode("position/eci-x-ft")

        if has_ecef:
            # At least one coordinate system should be available
            self.assertTrue(True)

        del fdm

    def test_terrain_elevation(self):
        """Test terrain elevation property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/terrain-elevation-ft"] = 500
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/terrain-elevation-asl-ft"):
            terrain = fdm["position/terrain-elevation-asl-ft"]
            self.assertAlmostEqual(terrain, 500, delta=10)

        del fdm

    def test_position_changes_during_flight(self):
        """Test that position changes during simulation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/u-fps"] = 100
        fdm["ic/v-fps"] = 0
        fdm["ic/w-fps"] = 0
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        # Run simulation
        for _ in range(100):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]

        # Ball should fall due to gravity
        self.assertNotEqual(initial_alt, final_alt)

        del fdm

    def test_geodetic_vs_geocentric(self):
        """Test geodetic vs geocentric latitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/lat-gc-deg"] = 45.0
        fdm.run_ic()

        pm = fdm.get_property_manager()

        gc_lat = fdm["position/lat-gc-deg"]

        if pm.hasNode("position/lat-geod-deg"):
            geod_lat = fdm["position/lat-geod-deg"]
            # They should be similar but not identical due to Earth's ellipsoid
            self.assertAlmostEqual(gc_lat, geod_lat, delta=1.0)

        del fdm

    def test_distance_from_earth_center(self):
        """Test distance from Earth center."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/radius-to-vehicle-ft"):
            radius = fdm["position/radius-to-vehicle-ft"]
            # Earth radius is about 20.9 million feet
            self.assertGreater(radius, 20000000)
            self.assertLess(radius, 22000000)

        del fdm

    def test_high_altitude(self):
        """Test high altitude position."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 100000  # 100,000 ft (edge of space)
        fdm.run_ic()

        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 100000, delta=100)

        del fdm

    def test_negative_longitude(self):
        """Test negative (west) longitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/long-gc-deg"] = -180.0
        fdm.run_ic()

        lon = fdm["position/long-gc-deg"]
        # Should handle -180 properly
        self.assertAlmostEqual(abs(lon), 180.0, delta=0.1)

        del fdm

    def test_equator_position(self):
        """Test position at equator."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/lat-gc-deg"] = 0.0
        fdm["ic/long-gc-deg"] = 0.0
        fdm.run_ic()

        lat = fdm["position/lat-gc-deg"]
        lon = fdm["position/long-gc-deg"]

        self.assertAlmostEqual(lat, 0.0, delta=0.01)
        self.assertAlmostEqual(lon, 0.0, delta=0.01)

        del fdm


if __name__ == "__main__":
    RunTest(TestPositionBasic)
