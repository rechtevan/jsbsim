# TestWeather.py
#
# Tests for weather and atmospheric conditions.
# Exercises temperature, pressure, and density properties.
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


class TestWeather(JSBSimTestCase):
    """
    Tests for weather and atmosphere properties.

    Tests cover:
    - Temperature properties
    - Pressure properties
    - Density properties
    - Standard atmosphere verification
    """

    def test_temperature_rankine(self):
        """Test temperature in Rankine."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        temp = fdm["atmosphere/T-R"]
        # Standard day sea level: ~518.67 R (59°F)
        self.assertAlmostEqual(temp, 518.67, delta=10, msg="Temp should be ~518 R")

        del fdm

    def test_temperature_decreases_with_altitude(self):
        """Test temperature decreases with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")

        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()
        temp_low = fdm["atmosphere/T-R"]

        fdm["ic/h-sl-ft"] = 30000
        fdm.run_ic()
        temp_high = fdm["atmosphere/T-R"]

        self.assertLess(temp_high, temp_low, "Temp should decrease with altitude")

        del fdm

    def test_pressure_sl(self):
        """Test pressure at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pressure = fdm["atmosphere/P-psf"]
        # Standard day sea level: ~2116.22 psf
        self.assertAlmostEqual(pressure, 2116.22, delta=50, msg="Pressure should be ~2116 psf")

        del fdm

    def test_pressure_decreases_with_altitude(self):
        """Test pressure decreases with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")

        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()
        p_low = fdm["atmosphere/P-psf"]

        fdm["ic/h-sl-ft"] = 20000
        fdm.run_ic()
        p_high = fdm["atmosphere/P-psf"]

        self.assertLess(p_high, p_low, "Pressure should decrease with altitude")

        del fdm

    def test_density_sl(self):
        """Test density at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        rho = fdm["atmosphere/rho-slugs_ft3"]
        # Standard day sea level: ~0.002377 slug/ft³
        self.assertAlmostEqual(rho, 0.002377, delta=0.0005, msg="Density should be ~0.002377")

        del fdm

    def test_density_decreases_with_altitude(self):
        """Test density decreases with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")

        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()
        rho_low = fdm["atmosphere/rho-slugs_ft3"]

        fdm["ic/h-sl-ft"] = 25000
        fdm.run_ic()
        rho_high = fdm["atmosphere/rho-slugs_ft3"]

        self.assertLess(rho_high, rho_low, "Density should decrease with altitude")

        del fdm

    def test_speed_of_sound(self):
        """Test speed of sound property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        a = fdm["atmosphere/a-fps"]
        # Standard day sea level: ~1116 fps
        self.assertAlmostEqual(a, 1116, delta=50, msg="Speed of sound should be ~1116")

        del fdm

    def test_sigma_property(self):
        """Test density ratio (sigma) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        sigma = fdm["atmosphere/sigma"]
        # At sea level, sigma should be 1.0
        self.assertAlmostEqual(sigma, 1.0, delta=0.01, msg="Sigma should be 1.0 at SL")

        del fdm


if __name__ == "__main__":
    RunTest(TestWeather)
