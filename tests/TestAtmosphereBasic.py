# TestAtmosphereBasic.py
#
# Comprehensive tests for basic atmospheric model functionality (ISA 1976)
# Tests temperature, pressure, density, and derived properties at various altitudes
#
# Copyright (c) 2025
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

import math

from JSBSim_utils import JSBSimTestCase, RunTest


class TestAtmosphereBasic(JSBSimTestCase):
    """
    Comprehensive tests for basic atmospheric model properties.

    Tests ISA 1976 standard atmosphere implementation including:
    - Sea level conditions
    - Pressure decrease with altitude
    - Temperature lapse rate in troposphere
    - Isothermal stratosphere
    - Speed of sound calculations
    - Atmospheric ratios (theta, sigma, delta)
    - Temperature offset effects
    """

    def setUp(self, *args):
        JSBSimTestCase.setUp(self, *args)

        # ISA 1976 standard constants (SI units)
        self.T0_K = 288.15  # K - Sea level temperature
        self.P0_Pa = 101325  # Pa - Sea level pressure
        self.rho0_kg_m3 = 1.225  # kg/m3 - Sea level density
        self.gamma = 1.4  # Specific heat ratio
        self.R_air = 287.05  # J/(kg*K) - Gas constant for air

        # ISA 1976 lapse rates
        self.troposphere_lapse_rate = -6.5  # K/km or -0.0019812 K/ft
        self.tropopause_altitude_ft = 36089.24  # ft (11 km)
        self.stratosphere_temp_K = 216.65  # K - Temperature above tropopause

        # Conversion factors (from TestStdAtmosphere.py)
        self.K_to_R = 1.8
        self.Pa_to_psf = 1.0 / 47.88
        self.kg_to_slug = 0.06852168
        self.m_to_ft = 1000 / 0.3048 / 1000  # meters to feet

        # Standard values in British units
        self.T0_R = self.T0_K * self.K_to_R  # 518.67 R
        self.P0_psf = self.P0_Pa * self.Pa_to_psf  # 2116.22 psf

    def test_sea_level_standard_atmosphere(self):
        """
        Test ISA standard conditions at sea level (0 ft).

        Verifies:
        - Temperature = 518.67 R (59 F, 15 C, 288.15 K)
        - Pressure = 2116.22 psf (29.92 inHg, 101325 Pa)
        - Density = 0.002377 slugs/ft3 (1.225 kg/m3)
        - Ratios (theta, sigma, delta) = 1.0
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 0.0
        fdm.run_ic()
        fdm.run()

        # Check temperature at sea level
        T_R = fdm["atmosphere/T-R"]
        self.assertAlmostEqual(
            T_R,
            self.T0_R,
            places=2,
            msg=f"Sea level temperature should be {self.T0_R} R, got {T_R} R",
        )

        # Check pressure at sea level
        P_psf = fdm["atmosphere/P-psf"]
        self.assertAlmostEqual(
            P_psf,
            self.P0_psf,
            places=1,
            msg=f"Sea level pressure should be {self.P0_psf} psf, got {P_psf} psf",
        )

        # Check density at sea level (approximately 0.002377 slugs/ft3)
        rho_slugs_ft3 = fdm["atmosphere/rho-slugs_ft3"]
        expected_rho = self.rho0_kg_m3 * self.kg_to_slug / (self.m_to_ft**3)
        self.assertAlmostEqual(
            rho_slugs_ft3,
            expected_rho,
            places=6,
            msg=f"Sea level density should be ~{expected_rho} slugs/ft3",
        )

        # Check atmospheric ratios are 1.0 at sea level
        self.assertAlmostEqual(
            fdm["atmosphere/theta"],
            1.0,
            places=5,
            msg="Temperature ratio (theta) should be 1.0 at sea level",
        )
        self.assertAlmostEqual(
            fdm["atmosphere/sigma"],
            1.0,
            places=5,
            msg="Density ratio (sigma) should be 1.0 at sea level",
        )
        self.assertAlmostEqual(
            fdm["atmosphere/delta"],
            1.0,
            places=5,
            msg="Pressure ratio (delta) should be 1.0 at sea level",
        )

    def test_pressure_altitude_relationship(self):
        """
        Test that pressure decreases monotonically with altitude.

        Tests altitudes from 0 to 50,000 ft and verifies:
        - Pressure decreases as altitude increases
        - Pressure altitude matches geometric altitude (standard conditions)
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        pressures = []
        altitudes = [0, 5000, 10000, 15000, 20000, 25000, 30000, 40000, 50000]

        for alt in altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            fdm.run()

            p = fdm["atmosphere/P-psf"]
            pressures.append(p)

            # Under standard conditions, pressure altitude should equal geometric altitude
            pressure_alt = fdm["atmosphere/pressure-altitude"]
            self.assertAlmostEqual(
                alt,
                pressure_alt,
                delta=0.1,
                msg=f"Pressure altitude should equal geometric altitude at {alt} ft under standard conditions",
            )

        # Verify pressure decreases monotonically with altitude
        for i in range(len(pressures) - 1):
            self.assertGreater(
                pressures[i],
                pressures[i + 1],
                msg=f"Pressure should decrease: at {altitudes[i]} ft = {pressures[i]} psf, "
                f"at {altitudes[i+1]} ft = {pressures[i+1]} psf",
            )

    def test_temperature_lapse_rate(self):
        """
        Test tropospheric temperature lapse rate.

        ISA 1976 specifies -6.5 K/km (-0.0019812 K/ft or -0.003566 R/ft).
        Verifies temperature decreases linearly with altitude below tropopause.
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test within troposphere (below ~36,000 ft)
        test_altitudes = [0, 5000, 10000, 15000, 20000, 25000, 30000, 35000]

        for i, alt in enumerate(test_altitudes[:-1]):
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            fdm.run()
            T1_R = fdm["atmosphere/T-R"]

            next_alt = test_altitudes[i + 1]
            fdm["ic/h-sl-ft"] = next_alt
            fdm.run_ic()
            fdm.run()
            T2_R = fdm["atmosphere/T-R"]

            # Temperature should decrease
            self.assertGreater(
                T1_R,
                T2_R,
                msg=f"Temperature should decrease with altitude: {T1_R} R at {alt} ft, {T2_R} R at {next_alt} ft",
            )

            # Calculate observed lapse rate in R/ft
            delta_alt_ft = next_alt - alt
            delta_T_R = T2_R - T1_R
            lapse_rate_R_per_ft = delta_T_R / delta_alt_ft

            # Expected lapse rate: -6.5 K/km = -0.003566 R/ft (approximately)
            expected_lapse_rate = -6.5 / 1000 * self.K_to_R / (1.0 / 0.3048)
            # Allow small tolerance due to geopotential altitude corrections
            self.assertAlmostEqual(
                lapse_rate_R_per_ft,
                expected_lapse_rate,
                places=4,
                msg=f"Lapse rate should be ~{expected_lapse_rate} R/ft in troposphere",
            )

    def test_stratosphere_isothermal(self):
        """
        Test isothermal layer in lower stratosphere.

        Above tropopause (~36,089 ft / 11 km), temperature remains constant
        at approximately 216.65 K (389.97 R) up to ~65,617 ft (20 km).
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test in lower stratosphere (above tropopause)
        stratosphere_altitudes = [37000, 40000, 45000, 50000, 55000, 60000, 65000]

        temperatures = []
        for alt in stratosphere_altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            fdm.run()

            T_R = fdm["atmosphere/T-R"]
            temperatures.append(T_R)

        # All temperatures should be approximately equal (isothermal)
        expected_T_R = self.stratosphere_temp_K * self.K_to_R
        for i, (alt, T_R) in enumerate(zip(stratosphere_altitudes, temperatures)):
            self.assertAlmostEqual(
                T_R,
                expected_T_R,
                delta=1.0,
                msg=f"Temperature at {alt} ft should be ~{expected_T_R} R (isothermal stratosphere)",
            )

        # Verify temperature variation is minimal (< 0.5 R)
        T_max = max(temperatures)
        T_min = min(temperatures)
        self.assertLess(
            T_max - T_min,
            0.5,
            msg="Temperature should be nearly constant in isothermal stratosphere",
        )

    def test_density_altitude(self):
        """
        Test density altitude calculation.

        Under standard conditions, density altitude should equal geometric altitude.
        With temperature offset, density altitude should differ appropriately.
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test 1: Standard conditions - density altitude = geometric altitude
        test_altitudes = [0, 5000, 10000, 20000, 30000]

        for alt in test_altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm["atmosphere/delta-T"] = 0.0  # Standard conditions
            fdm.run_ic()
            fdm.run()

            density_alt = fdm["atmosphere/density-altitude"]
            self.assertAlmostEqual(
                alt,
                density_alt,
                delta=0.1,
                msg=f"Density altitude should equal geometric altitude ({alt} ft) under standard conditions",
            )

        # Test 2: Hot day - density altitude > geometric altitude
        fdm["ic/h-sl-ft"] = 5000
        fdm["atmosphere/delta-T"] = 27.0  # +27 R (~15 K) hotter
        fdm.run_ic()
        fdm.run()

        density_alt_hot = fdm["atmosphere/density-altitude"]
        self.assertGreater(
            density_alt_hot,
            5000,
            msg="On hot day, density altitude should be higher than geometric altitude",
        )

        # Test 3: Cold day - density altitude < geometric altitude
        fdm["ic/h-sl-ft"] = 5000
        fdm["atmosphere/delta-T"] = -27.0  # -27 R (~-15 K) colder
        fdm.run_ic()
        fdm.run()

        density_alt_cold = fdm["atmosphere/density-altitude"]
        self.assertLess(
            density_alt_cold,
            5000,
            msg="On cold day, density altitude should be lower than geometric altitude",
        )

    def test_speed_of_sound(self):
        """
        Test speed of sound calculation: a = sqrt(gamma * R * T).

        Speed of sound varies with temperature.
        At sea level standard conditions: ~1116.45 ft/s
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm.run_ic()
        fdm.run()

        a_fps = fdm["atmosphere/a-fps"]
        a_sl_fps = fdm["atmosphere/a-sl-fps"]

        # Calculate expected speed of sound
        # a = sqrt(gamma * R * T) where R = 1716.56 ft*lbf/(slug*R)
        R_british = 1716.56  # ft*lbf/(slug*R)
        expected_a = math.sqrt(self.gamma * R_british * self.T0_R)

        self.assertAlmostEqual(
            a_fps,
            expected_a,
            delta=1.0,
            msg=f"Speed of sound at sea level should be ~{expected_a} ft/s",
        )
        self.assertAlmostEqual(
            a_fps,
            a_sl_fps,
            places=3,
            msg="Speed of sound should equal sea level value at sea level",
        )

        # Test at altitude - speed of sound decreases with temperature
        fdm["ic/h-sl-ft"] = 20000
        fdm.run_ic()
        fdm.run()

        a_20k_fps = fdm["atmosphere/a-fps"]
        T_20k_R = fdm["atmosphere/T-R"]

        expected_a_20k = math.sqrt(self.gamma * R_british * T_20k_R)
        self.assertAlmostEqual(
            a_20k_fps,
            expected_a_20k,
            delta=1.0,
            msg=f"Speed of sound at 20,000 ft should be ~{expected_a_20k} ft/s",
        )

        # Speed of sound at altitude should be less than at sea level (colder)
        self.assertLess(
            a_20k_fps,
            a_fps,
            msg="Speed of sound should decrease with altitude (temperature decreases)",
        )

    def test_atmosphere_at_high_altitude(self):
        """
        Test atmospheric properties at high altitude (50,000+ ft).

        Verifies calculations remain valid at extreme altitudes.
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        high_altitudes = [50000, 60000, 70000, 80000]

        for alt in high_altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            fdm.run()

            # All properties should be positive and physically reasonable
            T_R = fdm["atmosphere/T-R"]
            P_psf = fdm["atmosphere/P-psf"]
            rho = fdm["atmosphere/rho-slugs_ft3"]
            a_fps = fdm["atmosphere/a-fps"]

            self.assertGreater(T_R, 0.0, msg=f"Temperature at {alt} ft must be positive")
            self.assertGreater(P_psf, 0.0, msg=f"Pressure at {alt} ft must be positive")
            self.assertGreater(rho, 0.0, msg=f"Density at {alt} ft must be positive")
            self.assertGreater(a_fps, 0.0, msg=f"Speed of sound at {alt} ft must be positive")

            # Ratios should be less than 1.0 (lower than sea level)
            theta = fdm["atmosphere/theta"]
            sigma = fdm["atmosphere/sigma"]
            delta = fdm["atmosphere/delta"]

            self.assertLess(theta, 1.0, msg=f"Temperature ratio at {alt} ft should be < 1.0")
            self.assertLess(sigma, 1.0, msg=f"Density ratio at {alt} ft should be < 1.0")
            self.assertLess(delta, 1.0, msg=f"Pressure ratio at {alt} ft should be < 1.0")

    def test_temperature_offset(self):
        """
        Test delta-T capability for non-standard day.

        Setting atmosphere/delta-T should offset temperature uniformly.
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        test_altitude = 10000
        delta_T_R = 15.0  # +15 R offset

        # Get standard temperature at 10,000 ft
        fdm["ic/h-sl-ft"] = test_altitude
        fdm["atmosphere/delta-T"] = 0.0
        fdm.run_ic()
        fdm.run()
        T_standard_R = fdm["atmosphere/T-R"]

        # Apply temperature offset
        fdm["atmosphere/delta-T"] = delta_T_R
        fdm.run_ic()
        fdm.run()
        T_offset_R = fdm["atmosphere/T-R"]

        # Temperature should increase by delta-T
        self.assertAlmostEqual(
            T_offset_R,
            T_standard_R + delta_T_R,
            places=3,
            msg=f"Temperature with delta-T should be offset by {delta_T_R} R",
        )

        # Test negative offset
        delta_T_R_neg = -20.0
        fdm["atmosphere/delta-T"] = delta_T_R_neg
        fdm.run_ic()
        fdm.run()
        T_offset_neg_R = fdm["atmosphere/T-R"]

        self.assertAlmostEqual(
            T_offset_neg_R,
            T_standard_R + delta_T_R_neg,
            places=3,
            msg=f"Temperature with negative delta-T should be offset by {delta_T_R_neg} R",
        )

    def test_atmospheric_ratios(self):
        """
        Test atmospheric ratios (theta, sigma, delta).

        - theta = T / T_sl (temperature ratio)
        - sigma = rho / rho_sl (density ratio)
        - delta = P / P_sl (pressure ratio)

        These ratios are fundamental for aerodynamic calculations.
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        test_altitudes = [0, 10000, 20000, 30000, 40000]

        for alt in test_altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            fdm.run()

            # Get atmospheric properties
            T_R = fdm["atmosphere/T-R"]
            T_sl_R = fdm["atmosphere/T-sl-R"]
            rho = fdm["atmosphere/rho-slugs_ft3"]
            rho_sl = fdm["atmosphere/rho-sl-slugs_ft3"]
            P_psf = fdm["atmosphere/P-psf"]
            P_sl_psf = fdm["atmosphere/P-sl-psf"]

            # Get ratios from properties
            theta = fdm["atmosphere/theta"]
            sigma = fdm["atmosphere/sigma"]
            delta = fdm["atmosphere/delta"]

            # Calculate expected ratios
            expected_theta = T_R / T_sl_R
            expected_sigma = rho / rho_sl
            expected_delta = P_psf / P_sl_psf

            self.assertAlmostEqual(
                theta, expected_theta, places=5, msg=f"Theta ratio at {alt} ft should be T/T_sl"
            )
            self.assertAlmostEqual(
                sigma, expected_sigma, places=5, msg=f"Sigma ratio at {alt} ft should be rho/rho_sl"
            )
            self.assertAlmostEqual(
                delta, expected_delta, places=5, msg=f"Delta ratio at {alt} ft should be P/P_sl"
            )

            # At altitude, all ratios should be <= 1.0
            if alt > 0:
                self.assertLessEqual(theta, 1.0, msg=f"Theta should be <= 1.0 at altitude {alt} ft")
                self.assertLessEqual(sigma, 1.0, msg=f"Sigma should be <= 1.0 at altitude {alt} ft")
                self.assertLessEqual(delta, 1.0, msg=f"Delta should be <= 1.0 at altitude {alt} ft")

    def test_viscosity_derived_properties(self):
        """
        Test derived atmospheric properties.

        While viscosity is computed internally, we can verify that:
        - Reynolds number calculations are possible
        - Properties remain physically consistent
        """
        fdm = self.create_fdm()
        fdm.load_model("ball")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm.run_ic()
        fdm.run()

        T_R = fdm["atmosphere/T-R"]
        rho = fdm["atmosphere/rho-slugs_ft3"]
        P_psf = fdm["atmosphere/P-psf"]

        # Verify ideal gas law: P = rho * R * T
        R_british = 1716.56  # ft*lbf/(slug*R)
        P_calculated = rho * R_british * T_R

        self.assertAlmostEqual(
            P_psf,
            P_calculated,
            delta=1.0,
            msg="Pressure should satisfy ideal gas law P = rho * R * T",
        )

        # Test at altitude
        fdm["ic/h-sl-ft"] = 25000
        fdm.run_ic()
        fdm.run()

        T_R_alt = fdm["atmosphere/T-R"]
        rho_alt = fdm["atmosphere/rho-slugs_ft3"]
        P_psf_alt = fdm["atmosphere/P-psf"]

        P_calculated_alt = rho_alt * R_british * T_R_alt

        self.assertAlmostEqual(
            P_psf_alt,
            P_calculated_alt,
            delta=1.0,
            msg="Pressure should satisfy ideal gas law at altitude",
        )


RunTest(TestAtmosphereBasic)
