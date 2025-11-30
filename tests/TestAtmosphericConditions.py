# TestAtmosphericConditions.py
#
# Tests for atmospheric condition properties.
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


class TestAtmosphericConditions(JSBSimTestCase):
    """
    Tests for atmospheric condition properties.

    Tests cover:
    - Temperature
    - Pressure
    - Density
    - Speed of sound
    """

    def test_temperature_at_sea_level(self):
        """Test temperature at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/T-R"):
            temp = fdm["atmosphere/T-R"]
            # ISA sea level is 518.67 R (59 F)
            self.assertAlmostEqual(temp, 518.67, delta=20)

        del fdm

    def test_temperature_lapse_rate(self):
        """Test temperature decreases with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("atmosphere/T-R"):
            del fdm
            return

        temp_low = fdm["atmosphere/T-R"]

        fdm["ic/h-sl-ft"] = 20000
        fdm.run_ic()
        temp_high = fdm["atmosphere/T-R"]

        # Temperature should decrease with altitude (troposphere)
        self.assertLess(temp_high, temp_low)

        del fdm

    def test_pressure_at_sea_level(self):
        """Test pressure at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/P-psf"):
            pressure = fdm["atmosphere/P-psf"]
            # ISA sea level is 2116.2 psf
            self.assertAlmostEqual(pressure, 2116.2, delta=50)

        del fdm

    def test_pressure_decreases_with_altitude(self):
        """Test pressure decreases with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("atmosphere/P-psf"):
            del fdm
            return

        pressure_low = fdm["atmosphere/P-psf"]

        fdm["ic/h-sl-ft"] = 20000
        fdm.run_ic()
        pressure_high = fdm["atmosphere/P-psf"]

        self.assertLess(pressure_high, pressure_low)

        del fdm

    def test_density_at_sea_level(self):
        """Test density at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/rho-slugs_ft3"):
            rho = fdm["atmosphere/rho-slugs_ft3"]
            # ISA sea level is 0.002377 slugs/ft3
            self.assertAlmostEqual(rho, 0.002377, delta=0.0005)

        del fdm

    def test_speed_of_sound(self):
        """Test speed of sound property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/a-fps"):
            a = fdm["atmosphere/a-fps"]
            # Sea level speed of sound is about 1116 fps
            self.assertAlmostEqual(a, 1116, delta=20)

        del fdm

    def test_sigma_density_ratio(self):
        """Test density ratio (sigma) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/sigma"):
            sigma = fdm["atmosphere/sigma"]
            # At sea level, sigma = 1.0
            self.assertAlmostEqual(sigma, 1.0, delta=0.05)

        del fdm


if __name__ == "__main__":
    RunTest(TestAtmosphericConditions)
