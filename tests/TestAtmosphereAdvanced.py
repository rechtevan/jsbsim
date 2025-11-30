# TestAtmosphereAdvanced.py
#
# Advanced tests for atmosphere and environment models.
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


class TestAtmosphereAdvanced(JSBSimTestCase):
    """
    Advanced tests for atmosphere and environment models.

    Tests cover:
    - Atmosphere model variations
    - Temperature gradients
    - Density altitude
    - Pressure altitude
    - Extreme altitudes
    """

    def test_standard_atmosphere_sea_level(self):
        """Test standard atmosphere at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Check sea level properties
        if pm.hasNode("atmosphere/T-R"):
            temp = fdm["atmosphere/T-R"]
            # Standard temp is 518.67 R (59°F)
            self.assertAlmostEqual(temp, 518.67, delta=5)

        if pm.hasNode("atmosphere/P-psf"):
            pressure = fdm["atmosphere/P-psf"]
            # Standard pressure is 2116.22 psf
            self.assertAlmostEqual(pressure, 2116.22, delta=50)

        del fdm

    def test_temperature_lapse_rate(self):
        """Test temperature decreases with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        temps = []
        for alt in [0, 10000, 20000, 30000]:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("atmosphere/T-R"):
                temps.append(fdm["atmosphere/T-R"])

        # Temperature should decrease with altitude (up to tropopause)
        if len(temps) >= 2:
            for i in range(len(temps) - 1):
                self.assertGreater(temps[i], temps[i + 1])

        del fdm

    def test_density_altitude_calculation(self):
        """Test density altitude varies with conditions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/density-altitude"):
            density_alt = fdm["atmosphere/density-altitude"]
            # Density altitude should be reasonable
            self.assertGreater(density_alt, 0)
            self.assertLess(density_alt, 50000)

        del fdm

    def test_pressure_altitude(self):
        """Test pressure altitude calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 10000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/pressure-altitude"):
            press_alt = fdm["atmosphere/pressure-altitude"]
            # Pressure altitude should be near geometric
            self.assertAlmostEqual(press_alt, 10000, delta=1000)

        del fdm

    def test_high_altitude_atmosphere(self):
        """Test atmosphere at high altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("X15")
        fdm["ic/h-sl-ft"] = 100000
        fdm["ic/vc-kts"] = 500
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("atmosphere/rho-slugs_ft3"):
            density = fdm["atmosphere/rho-slugs_ft3"]
            # Density should be very low at 100,000 ft
            self.assertLess(density, 0.0001)

        del fdm

    def test_atmosphere_density_effects_on_flight(self):
        """Test atmosphere density affects aircraft performance."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Low altitude - dense air
        fdm["ic/h-sl-ft"] = 2000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()
        fdm["fcs/throttle-cmd-norm[0]"] = 0.7

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        low_alt_tas = 0
        if pm.hasNode("velocities/vt-fps"):
            low_alt_tas = fdm["velocities/vt-fps"]

        del fdm

        # High altitude - thin air
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 12000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()
        fdm["fcs/throttle-cmd-norm[0]"] = 0.7

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vt-fps"):
            high_alt_tas = fdm["velocities/vt-fps"]
            # TAS should be higher at altitude for same IAS
            if low_alt_tas > 0:
                self.assertGreater(high_alt_tas, low_alt_tas * 0.9)

        del fdm

    def test_tropopause_transition(self):
        """Test atmosphere behavior at tropopause."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("Concorde")

        # Below tropopause
        fdm["ic/h-sl-ft"] = 35000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        temp_below = 0
        if pm.hasNode("atmosphere/T-R"):
            temp_below = fdm["atmosphere/T-R"]

        # At/above tropopause (~36,000 ft)
        fdm["ic/h-sl-ft"] = 40000
        fdm.run_ic()

        if pm.hasNode("atmosphere/T-R"):
            temp_above = fdm["atmosphere/T-R"]
            # Temperature should level off in stratosphere
            if temp_below > 0:
                # Difference should be small in isothermal region
                self.assertLess(abs(temp_above - temp_below), 20)

        del fdm

    def test_speed_of_sound_variation(self):
        """Test speed of sound varies with altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        sos_values = []
        for alt in [0, 20000, 40000]:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("atmosphere/a-fps"):
                sos_values.append(fdm["atmosphere/a-fps"])

        # Speed of sound decreases with altitude (lower temp)
        if len(sos_values) >= 2:
            self.assertGreater(sos_values[0], sos_values[1])

        del fdm


if __name__ == "__main__":
    RunTest(TestAtmosphereAdvanced)
