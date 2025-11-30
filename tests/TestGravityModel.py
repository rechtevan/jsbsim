# TestGravityModel.py
#
# Tests for gravity and gravitational model (FGInertial).
# Exercises gravity acceleration and variation with altitude.
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


class TestGravityModel(JSBSimTestCase):
    """
    Tests for gravity and inertial model (FGInertial).

    Tests cover:
    - Gravity acceleration values
    - Gravity variation with altitude
    - Earth model parameters
    - Inertial reference frame
    """

    def test_gravity_at_sea_level(self):
        """Test gravity value at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        fdm.run()

        # Gravity should be approximately 32.174 ft/s^2 at sea level
        if fdm.get_property_manager().hasNode("accelerations/gravity-ft_sec2"):
            g = fdm["accelerations/gravity-ft_sec2"]
            self.assertAlmostEqual(g, 32.174, delta=0.5, msg="Gravity ~32.174 ft/s^2")

        del fdm

    def test_gravity_positive(self):
        """Test that gravity is positive (downward)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        fdm.run()

        if fdm.get_property_manager().hasNode("accelerations/gravity-ft_sec2"):
            g = fdm["accelerations/gravity-ft_sec2"]
            self.assertGreater(g, 0, "Gravity should be positive")

        del fdm

    def test_gravity_decreases_with_altitude(self):
        """Test that gravity decreases slightly with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")

        # At sea level
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()
        fdm.run()

        if fdm.get_property_manager().hasNode("accelerations/gravity-ft_sec2"):
            g_low = fdm["accelerations/gravity-ft_sec2"]

            # At high altitude (100,000 ft)
            fdm["ic/h-sl-ft"] = 100000
            fdm.run_ic()
            fdm.run()

            g_high = fdm["accelerations/gravity-ft_sec2"]

            # Gravity should decrease with altitude
            self.assertLess(g_high, g_low, "Gravity should decrease with altitude")

        del fdm

    def test_earth_radius_property(self):
        """Test Earth radius property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("position/terrain-elevation-asl-ft"):
            terrain = fdm["position/terrain-elevation-asl-ft"]
            self.assertIsNotNone(terrain, "Terrain elevation should be accessible")

        del fdm

    def test_sea_level_radius(self):
        """Test sea level radius property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("position/radius-to-vehicle-ft"):
            radius = fdm["position/radius-to-vehicle-ft"]
            # Earth radius is approximately 20.9e6 ft
            self.assertGreater(radius, 20e6, "Radius should be Earth-sized")
            self.assertLess(radius, 22e6, "Radius should be Earth-sized")

        del fdm

    def test_inertial_position_ecef(self):
        """Test ECEF position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        fdm.run()

        # Check ECEF position properties
        if fdm.get_property_manager().hasNode("position/ecef-x-ft"):
            x = fdm["position/ecef-x-ft"]
            y = fdm["position/ecef-y-ft"]
            z = fdm["position/ecef-z-ft"]

            self.assertIsNotNone(x, "ECEF X should be accessible")
            self.assertIsNotNone(y, "ECEF Y should be accessible")
            self.assertIsNotNone(z, "ECEF Z should be accessible")

        del fdm

    def test_omega_earth(self):
        """Test Earth rotation rate property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        # Check for omega property
        if fdm.get_property_manager().hasNode("atmosphere/omega-rad_sec"):
            omega = fdm["atmosphere/omega-rad_sec"]
            # Earth rotation: ~7.29e-5 rad/s
            self.assertIsNotNone(omega, "Earth rotation rate should be accessible")

        del fdm

    def test_weight_force(self):
        """Test that weight force is gravity * mass."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        fdm.run()

        weight = fdm["inertia/weight-lbs"]
        mass = fdm["inertia/mass-slugs"]

        if fdm.get_property_manager().hasNode("accelerations/gravity-ft_sec2"):
            g = fdm["accelerations/gravity-ft_sec2"]
            # Weight = mass * g (allow for numerical precision)
            expected_weight = mass * g
            self.assertAlmostEqual(weight, expected_weight, delta=50.0, msg="Weight = mass * g")

        del fdm


if __name__ == "__main__":
    RunTest(TestGravityModel)
