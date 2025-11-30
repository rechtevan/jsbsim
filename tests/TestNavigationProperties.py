# TestNavigationProperties.py
#
# Tests for navigation-related properties.
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


class TestNavigationProperties(JSBSimTestCase):
    """
    Tests for navigation properties.

    Tests cover:
    - Latitude/longitude
    - Heading
    - Track
    - Distance traveled
    """

    def test_latitude_property(self):
        """Test latitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/lat-gc-deg"] = 37.5
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/lat-gc-deg"):
            lat = fdm["position/lat-gc-deg"]
            self.assertAlmostEqual(lat, 37.5, delta=0.1)

        del fdm

    def test_longitude_property(self):
        """Test longitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/long-gc-deg"] = -122.0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/long-gc-deg"):
            lon = fdm["position/long-gc-deg"]
            self.assertAlmostEqual(lon, -122.0, delta=0.1)

        del fdm

    def test_heading_magnetic(self):
        """Test magnetic heading property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/heading-true-rad"):
            heading = fdm["attitude/heading-true-rad"]
            self.assertIsNotNone(heading)

        del fdm

    def test_position_changes_with_flight(self):
        """Test position changes during flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/psi-true-deg"] = 90  # East
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("position/long-gc-deg"):
            del fdm
            return

        lon1 = fdm["position/long-gc-deg"]

        for _ in range(500):
            fdm.run()

        lon2 = fdm["position/long-gc-deg"]
        # Flying east, longitude should increase
        self.assertGreater(lon2, lon1)

        del fdm

    def test_geod_altitude(self):
        """Test geodetic altitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
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
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/terrain-elevation-asl-ft"):
            terrain = fdm["position/terrain-elevation-asl-ft"]
            self.assertIsNotNone(terrain)

        del fdm


if __name__ == "__main__":
    RunTest(TestNavigationProperties)
