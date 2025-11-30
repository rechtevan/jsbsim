# TestGeodeticPosition.py
#
# Tests for geodetic position (lat, lon, alt).
# Exercises FGLocation position properties.
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


class TestGeodeticPosition(JSBSimTestCase):
    """
    Tests for geodetic position.

    Tests cover:
    - Latitude properties
    - Longitude properties
    - Altitude properties
    - Position initialization
    - Position change over time
    """

    def test_latitude_gc_deg(self):
        """Test geocentric latitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/lat-gc-deg"] = 37.0
        fdm.run_ic()

        lat = fdm["position/lat-gc-deg"]
        self.assertAlmostEqual(lat, 37.0, delta=1.0, msg="Lat should be ~37")

        del fdm

    def test_longitude_gc_deg(self):
        """Test longitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/long-gc-deg"] = -122.0
        fdm.run_ic()

        lon = fdm["position/long-gc-deg"]
        self.assertAlmostEqual(lon, -122.0, delta=1.0, msg="Lon should be ~-122")

        del fdm

    def test_altitude_agl(self):
        """Test altitude AGL property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-agl-ft"] = 1000
        fdm.run_ic()

        h_agl = fdm["position/h-agl-ft"]
        self.assertAlmostEqual(h_agl, 1000, delta=100, msg="AGL should be ~1000")

        del fdm

    def test_altitude_sl(self):
        """Test altitude MSL property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        h_sl = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(h_sl, 5000, delta=100, msg="MSL should be ~5000")

        del fdm

    def test_geod_altitude(self):
        """Test geodetic altitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("position/geod-alt-ft"):
            h_geod = fdm["position/geod-alt-ft"]
            self.assertIsNotNone(h_geod, "Geod alt should be accessible")

        del fdm

    def test_terrain_elevation(self):
        """Test terrain elevation property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        terrain = fdm["position/terrain-elevation-asl-ft"]
        self.assertIsNotNone(terrain, "Terrain elevation should be accessible")

        del fdm

    def test_lat_geod_deg(self):
        """Test geodetic latitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/lat-geod-deg"] = 37.0
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("position/lat-geod-deg"):
            lat = fdm["position/lat-geod-deg"]
            self.assertAlmostEqual(lat, 37.0, delta=1.0, msg="Geod lat should be ~37")

        del fdm

    def test_position_changes_in_flight(self):
        """Test that position changes during flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/psi-true-deg"] = 90.0  # East
        fdm.run_ic()

        initial_lon = fdm["position/long-gc-deg"]

        # Fly for a while
        for _ in range(500):
            fdm.run()

        final_lon = fdm["position/long-gc-deg"]

        # Should have moved east
        self.assertGreater(final_lon, initial_lon, "Should move east")

        del fdm

    def test_radius_to_vehicle(self):
        """Test radius to vehicle property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("position/radius-to-vehicle-ft"):
            radius = fdm["position/radius-to-vehicle-ft"]
            # Should be approximately Earth radius + altitude
            self.assertGreater(radius, 20e6, "Radius should be Earth-sized")

        del fdm

    def test_distance_from_ref(self):
        """Test distance from reference point."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Check distance properties if available
        if fdm.get_property_manager().hasNode("position/distance-from-start-lat-ft"):
            dist = fdm["position/distance-from-start-lat-ft"]
            self.assertIsNotNone(dist, "Distance should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestGeodeticPosition)
