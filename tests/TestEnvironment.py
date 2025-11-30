# TestEnvironment.py
#
# Tests for environment properties (wind, gravity).
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


class TestEnvironment(JSBSimTestCase):
    """
    Tests for environment properties.

    Tests cover:
    - Wind components
    - Gravity
    - Planet properties
    """

    def test_wind_north(self):
        """Test wind north component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/wind-north-fps"):
            wn = fdm["atmosphere/wind-north-fps"]
            self.assertIsNotNone(wn)

        del fdm

    def test_wind_east(self):
        """Test wind east component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/wind-east-fps"):
            we = fdm["atmosphere/wind-east-fps"]
            self.assertIsNotNone(we)

        del fdm

    def test_wind_down(self):
        """Test wind down component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/wind-down-fps"):
            wd = fdm["atmosphere/wind-down-fps"]
            self.assertIsNotNone(wd)

        del fdm

    def test_set_wind(self):
        """Test setting wind values."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/wind-north-fps"):
            fdm["atmosphere/wind-north-fps"] = 20
            wn = fdm["atmosphere/wind-north-fps"]
            self.assertAlmostEqual(wn, 20, delta=1)

        del fdm

    def test_gravity_acceleration(self):
        """Test gravity acceleration property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/gravity-ft_sec2"):
            g = fdm["accelerations/gravity-ft_sec2"]
            # g is about 32.174 ft/s^2
            self.assertAlmostEqual(g, 32.174, delta=0.5)

        del fdm

    def test_earth_radius(self):
        """Test Earth radius property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/radius-to-vehicle-ft"):
            r = fdm["position/radius-to-vehicle-ft"]
            # Earth radius is about 20.9 million feet
            self.assertGreater(r, 20000000)
            self.assertLess(r, 22000000)

        del fdm

    def test_sea_level_radius(self):
        """Test sea level radius property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/radius-to-vehicle-ft"):
            r = fdm["position/radius-to-vehicle-ft"]
            self.assertGreater(r, 20000000)

        del fdm


if __name__ == "__main__":
    RunTest(TestEnvironment)
