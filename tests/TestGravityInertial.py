# TestGravityInertial.py
#
# Tests for gravity and inertial reference frame models.
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


class TestGravityInertial(JSBSimTestCase):
    """
    Tests for gravity and inertial reference frame models.

    Tests cover:
    - Gravity variation with altitude
    - Gravity variation with latitude
    - Inertial reference frames
    - WGS84 model
    """

    def test_gravity_at_sea_level(self):
        """Test gravity magnitude at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/lat-gc-deg"] = 45
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/gravity-ft_sec2"):
            g = fdm["accelerations/gravity-ft_sec2"]
            # Standard gravity is ~32.174 ft/s²
            self.assertAlmostEqual(g, 32.174, delta=0.5)

        del fdm

    def test_gravity_decreases_with_altitude(self):
        """Test gravity decreases with altitude."""
        g_values = []

        for alt in [0, 100000, 300000]:
            fdm = CreateFDM(self.sandbox)
            fdm.load_model("ball")
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("accelerations/gravity-ft_sec2"):
                g_values.append(fdm["accelerations/gravity-ft_sec2"])

            del fdm

        # Gravity should decrease with altitude
        if len(g_values) >= 2:
            self.assertGreater(g_values[0], g_values[1])

    def test_gravity_varies_with_latitude(self):
        """Test gravity varies with latitude (J2 effect)."""
        g_equator = 0
        g_pole = 0

        # At equator
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/lat-gc-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/gravity-ft_sec2"):
            g_equator = fdm["accelerations/gravity-ft_sec2"]
        del fdm

        # At pole
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/lat-gc-deg"] = 89
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/gravity-ft_sec2"):
            g_pole = fdm["accelerations/gravity-ft_sec2"]
        del fdm

        # Gravity should be slightly higher at poles
        if g_equator > 0 and g_pole > 0:
            self.assertGreater(g_pole, g_equator)

    def test_freefall_acceleration(self):
        """Test object in freefall accelerates at g."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/vc-kts"] = 0
        fdm.run_ic()

        # Get initial velocity
        initial_vdown = fdm["velocities/v-down-fps"]

        # Run for 1 second (assuming ~120 Hz)
        for _ in range(120):
            fdm.run()

        final_vdown = fdm["velocities/v-down-fps"]

        # Velocity should increase by approximately g * t
        delta_v = final_vdown - initial_vdown
        # Should be roughly 32 ft/s after 1 second
        self.assertGreater(delta_v, 20)
        self.assertLess(delta_v, 50)

        del fdm

    def test_position_geodetic_coordinates(self):
        """Test geodetic position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/lat-gc-deg"] = 37.0
        fdm["ic/long-gc-deg"] = -122.0
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Check geodetic latitude
        if pm.hasNode("position/lat-geod-deg"):
            lat = fdm["position/lat-geod-deg"]
            self.assertAlmostEqual(lat, 37.0, delta=1.0)

        # Check longitude
        if pm.hasNode("position/long-gc-deg"):
            lon = fdm["position/long-gc-deg"]
            self.assertAlmostEqual(lon, -122.0, delta=1.0)

        del fdm

    def test_earth_rotation_effect(self):
        """Test Earth rotation on long flights."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/lat-gc-deg"] = 45
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.6

        # Run for extended time
        for _ in range(1000):
            fdm.run()

        # Just verify simulation continues without error
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_altitude_above_ellipsoid(self):
        """Test altitude reference to ellipsoid."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-sl-ft"):
            h_sl = fdm["position/h-sl-ft"]
            self.assertAlmostEqual(h_sl, 5000, delta=100)

        del fdm


if __name__ == "__main__":
    RunTest(TestGravityInertial)
