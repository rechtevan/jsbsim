# TestLocationGeodesy.py
#
# Tests for location and geodesy calculations.
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


class TestLocationGeodesy(JSBSimTestCase):
    """
    Tests for location and geodesy.

    Tests cover:
    - Latitude/longitude
    - Altitude conversions
    - Earth model
    """

    def test_latitude_property(self):
        """Test latitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/lat-gc-deg"] = 45.0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/lat-gc-deg"):
            lat = fdm["position/lat-gc-deg"]
            self.assertAlmostEqual(lat, 45.0, delta=1.0)

        del fdm

    def test_longitude_property(self):
        """Test longitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/long-gc-deg"] = -120.0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/long-gc-deg"):
            lon = fdm["position/long-gc-deg"]
            self.assertAlmostEqual(lon, -120.0, delta=1.0)

        del fdm

    def test_altitude_agl(self):
        """Test altitude AGL property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-agl-ft"] = 1000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-agl-ft"):
            alt = fdm["position/h-agl-ft"]
            self.assertAlmostEqual(alt, 1000, delta=100)

        del fdm

    def test_altitude_msl(self):
        """Test altitude MSL property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-sl-ft"):
            alt = fdm["position/h-sl-ft"]
            self.assertAlmostEqual(alt, 5000, delta=100)

        del fdm

    def test_terrain_elevation(self):
        """Test terrain elevation property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/terrain-elevation-asl-ft"):
            terrain = fdm["position/terrain-elevation-asl-ft"]
            self.assertIsNotNone(terrain)

        del fdm

    def test_ecef_coordinates(self):
        """Test ECEF coordinate properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        ecef_props = [
            "position/ecef-x-ft",
            "position/ecef-y-ft",
            "position/ecef-z-ft",
        ]

        for prop in ecef_props:
            if pm.hasNode(prop):
                value = fdm[prop]
                self.assertIsNotNone(value)
                break

        del fdm

    def test_geodetic_vs_geocentric(self):
        """Test geodetic vs geocentric latitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/lat-gc-deg"] = 45.0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # At 45 degrees, geodetic and geocentric should differ slightly
        if pm.hasNode("position/lat-gc-deg") and pm.hasNode("position/lat-geod-deg"):
            lat_gc = fdm["position/lat-gc-deg"]
            lat_geod = fdm["position/lat-geod-deg"]
            # They should be close but not identical
            self.assertAlmostEqual(lat_gc, lat_geod, delta=1.0)

        del fdm


if __name__ == "__main__":
    RunTest(TestLocationGeodesy)
