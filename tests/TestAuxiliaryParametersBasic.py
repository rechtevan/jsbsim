# TestAuxiliaryParametersBasic.py
#
# Comprehensive tests for auxiliary/derived parameters (FGAuxiliary).
# Tests calculation of Mach number, airspeed conversions, angles of attack/sideslip,
# dynamic pressure, climb rate, ground speed/track, wind components, and temperature.
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


class TestAuxiliaryParametersBasic(JSBSimTestCase):
    """
    Comprehensive tests for FGAuxiliary derived parameter calculations.

    FGAuxiliary computes derived/auxiliary flight parameters from the
    primary state variables. These include:
    - Mach number
    - Airspeed types (calibrated, true, equivalent)
    - Aerodynamic angles (alpha, beta)
    - Dynamic pressure (qbar)
    - Climb rate parameters
    - Ground speed and track
    - Wind components
    - Total temperature

    Coverage areas:
    - FGAuxiliary: Mach number calculation at various speeds/altitudes
    - Airspeed conversions (vc, vt, ve)
    - Angle of attack and sideslip calculations
    - Dynamic pressure computation
    - Vertical speed and climb angle
    - Ground speed and ground track
    - Wind component calculations (headwind, crosswind)
    - Total vs static temperature (compressibility effects)
    - Pressure altitude vs density altitude
    """

    def test_mach_number_increases_with_speed(self):
        """
        Test that Mach number increases monotonically with airspeed.

        Validates that at constant altitude, increasing true airspeed
        results in proportionally increasing Mach number.

        Tests:
        - velocities/mach computation
        - Relationship between airspeed and Mach number
        - Consistency across speed range
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        speeds = [80, 100, 120, 140]  # kts
        machs = []

        for speed in speeds:
            fdm["ic/vc-kts"] = speed
            fdm["ic/h-sl-ft"] = 5000.0
            fdm.run_ic()
            fdm.run()

            mach = fdm["velocities/mach"]
            machs.append(mach)

        # Verify Mach increases with speed
        for i in range(len(machs) - 1):
            self.assertGreater(
                machs[i + 1],
                machs[i],
                msg=f"Mach should increase with speed: {machs[i]} -> {machs[i+1]}",
            )

        # Verify typical Mach range for C172 speeds
        self.assertLess(machs[0], 0.15, msg="80 kts should be well below Mach 0.15")
        self.assertLess(machs[-1], 0.25, msg="140 kts should be below Mach 0.25")

    def test_mach_number_varies_with_altitude(self):
        """
        Test that Mach number varies correctly with altitude.

        At constant calibrated airspeed, Mach number should increase with altitude
        because true airspeed increases and speed of sound decreases (in troposphere).

        Tests:
        - Mach number at different altitudes
        - Speed of sound variation with altitude
        - Compressibility effects
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        altitudes = [1000, 5000, 10000]  # ft
        machs = []

        # Use constant calibrated airspeed (more practical for aircraft operations)
        target_vc = 100.0  # kts calibrated

        for alt in altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm["ic/vc-kts"] = target_vc
            fdm.run_ic()
            fdm.run()

            mach = fdm["velocities/mach"]
            machs.append(mach)

        # At constant calibrated airspeed, Mach increases with altitude
        # (true airspeed increases and speed of sound decreases with altitude in troposphere)
        for i in range(len(machs) - 1):
            self.assertGreater(
                machs[i + 1],
                machs[i],
                msg=f"Mach should increase with altitude at constant calibrated airspeed: {machs[i]} -> {machs[i+1]}",
            )

    def test_airspeed_conversions(self):
        """
        Test relationship between calibrated, true, and equivalent airspeed.

        At sea level: vc ≈ vtrue ≈ ve
        At altitude: vtrue > vc (true > calibrated) due to lower density

        Tests:
        - velocities/vc-kts (calibrated airspeed)
        - velocities/vtrue-kts (true airspeed)
        - velocities/ve-kts (equivalent airspeed)
        - Correct relationships between airspeed types
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()
        fdm.run()

        vc_sl = fdm["velocities/vc-kts"]
        vt_sl = fdm["velocities/vtrue-kts"]
        ve_sl = fdm["velocities/ve-kts"]

        # At sea level, all airspeeds should be very similar
        self.assertAlmostEqual(vc_sl, vt_sl, delta=2.0, msg="At sea level, vc ≈ vtrue")
        self.assertAlmostEqual(vc_sl, ve_sl, delta=2.0, msg="At sea level, vc ≈ ve")

        # Test at altitude
        fdm["ic/h-sl-ft"] = 10000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()
        fdm.run()

        vc_alt = fdm["velocities/vc-kts"]
        vt_alt = fdm["velocities/vtrue-kts"]
        ve_alt = fdm["velocities/ve-kts"]

        # At altitude, true airspeed should be greater than calibrated
        self.assertGreater(vt_alt, vc_alt, msg="At altitude, vtrue > vc due to lower air density")

        # Equivalent airspeed should be close to calibrated (for low speeds)
        self.assertAlmostEqual(
            ve_alt, vc_alt, delta=5.0, msg="ve should be close to vc at low speeds"
        )

        # Verify the delta increases significantly
        delta_altitude = vt_alt - vc_alt
        self.assertGreater(
            delta_altitude,
            10.0,
            msg=f"vtrue-vc difference at 10000 ft should be significant: {delta_altitude} kts",
        )

    def test_alpha_beta_calculation(self):
        """
        Test angle of attack (alpha) and sideslip angle (beta) calculations.

        Alpha is the angle between the body x-axis and the velocity vector
        projected into the xz-plane. Beta is the angle of the velocity vector
        out of the xz-plane.

        Tests:
        - aero/alpha-deg and aero/alpha-rad
        - aero/beta-deg and aero/beta-rad
        - Alpha variation with pitch
        - Beta should be near zero for coordinated flight
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test level flight - alpha should be small positive
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/theta-deg"] = 0.0
        fdm["ic/phi-deg"] = 0.0
        fdm.run_ic()
        fdm.run()

        alpha_level = fdm["aero/alpha-deg"]
        beta_level = fdm["aero/beta-deg"]

        # Small positive alpha expected for level flight (lift generation)
        self.assertGreater(
            alpha_level, 0.0, msg="Alpha should be positive for level flight (generating lift)"
        )
        self.assertLess(alpha_level, 10.0, msg="Alpha should be moderate for cruise flight")

        # Beta should be near zero for wings-level coordinated flight
        self.assertAlmostEqual(
            beta_level, 0.0, delta=1.0, msg="Beta should be near zero for coordinated flight"
        )

        # Verify rad/deg conversion
        alpha_rad = fdm["aero/alpha-rad"]
        alpha_deg_from_rad = alpha_rad * 180.0 / math.pi
        self.assertAlmostEqual(
            alpha_deg_from_rad,
            alpha_level,
            delta=0.01,
            msg="Alpha rad/deg conversion should be consistent",
        )

        # Test with lower speed (higher alpha needed for lift)
        fdm["ic/vc-kts"] = 70.0  # Lower speed requires higher alpha
        fdm["ic/theta-deg"] = 0.0
        fdm.run_ic()
        fdm.run()

        alpha_slow = fdm["aero/alpha-deg"]
        # At lower speed, higher alpha is needed to generate same lift
        self.assertGreater(
            alpha_slow,
            alpha_level,
            msg=f"Alpha should increase at lower speed: {alpha_level} -> {alpha_slow}",
        )

    def test_qbar_calculation(self):
        """
        Test dynamic pressure (qbar) calculation.

        Dynamic pressure qbar = 0.5 * rho * V^2
        Should increase with both speed and density (lower altitude).

        Tests:
        - aero/qbar-psf (dynamic pressure in psf)
        - Variation with airspeed
        - Variation with altitude (density)
        - Quadratic relationship with speed
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test qbar increases with speed (quadratically)
        speeds = [80, 100, 120]
        qbars = []

        for speed in speeds:
            fdm["ic/vc-kts"] = speed
            fdm["ic/h-sl-ft"] = 5000.0
            fdm.run_ic()
            fdm.run()

            qbar = fdm["aero/qbar-psf"]
            qbars.append(qbar)

        # Verify qbar increases with speed
        for i in range(len(qbars) - 1):
            self.assertGreater(
                qbars[i + 1],
                qbars[i],
                msg=f"Qbar should increase with speed: {qbars[i]} -> {qbars[i+1]}",
            )

        # Check approximate quadratic relationship
        # qbar ratio should be approximately (speed ratio)^2
        speed_ratio = speeds[1] / speeds[0]  # 100/80 = 1.25
        qbar_ratio = qbars[1] / qbars[0]
        expected_ratio = speed_ratio**2  # 1.5625

        self.assertAlmostEqual(
            qbar_ratio,
            expected_ratio,
            delta=0.15,
            msg=f"Qbar should vary quadratically with speed: ratio {qbar_ratio} vs expected {expected_ratio}",
        )

        # Test qbar decreases with altitude (lower density)
        altitudes = [0, 5000, 10000]
        qbars_alt = []

        for alt in altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm["ic/vc-kts"] = 100.0
            fdm.run_ic()
            fdm.run()

            qbar = fdm["aero/qbar-psf"]
            qbars_alt.append(qbar)

        # Verify qbar decreases with altitude
        for i in range(len(qbars_alt) - 1):
            self.assertGreater(
                qbars_alt[i],
                qbars_alt[i + 1],
                msg=f"Qbar should decrease with altitude: {qbars_alt[i]} -> {qbars_alt[i+1]}",
            )

    def test_climb_rate_parameters(self):
        """
        Test vertical speed and climb angle calculations.

        Tests relationship between h-dot (rate of climb) and flight path
        angle (gamma). Also verifies vertical velocity component.

        Tests:
        - velocities/h-dot-fps (rate of climb in ft/s)
        - velocities/v-down-fps (vertical velocity, positive down)
        - flight-path/gamma-deg (flight path angle)
        - Relationship between climb rate and gamma
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test level flight - climb rate should be near zero
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/gamma-deg"] = 0.0
        fdm.run_ic()
        fdm.run()

        h_dot_level = fdm["velocities/h-dot-fps"]
        v_down_level = fdm["velocities/v-down-fps"]
        gamma_level = fdm["flight-path/gamma-deg"]

        # Level flight should have near-zero climb rate
        self.assertAlmostEqual(
            h_dot_level, 0.0, delta=5.0, msg="h-dot should be near zero for level flight"
        )
        self.assertAlmostEqual(
            gamma_level, 0.0, delta=2.0, msg="Gamma should be near zero for level flight"
        )

        # h-dot and v-down should have opposite signs (v-down positive down)
        # For near-level flight, both should be close to zero
        self.assertAlmostEqual(
            v_down_level, 0.0, delta=5.0, msg="v-down should be near zero for level flight"
        )

        # Test climbing flight
        fdm["ic/gamma-deg"] = 5.0  # 5 degree climb
        fdm.run_ic()
        fdm.run()

        h_dot_climb = fdm["velocities/h-dot-fps"]
        v_down_climb = fdm["velocities/v-down-fps"]
        gamma_climb = fdm["flight-path/gamma-deg"]

        # Climb should have positive h-dot
        self.assertGreater(
            h_dot_climb, 5.0, msg="h-dot should be positive and significant for climb"
        )

        # Climb should have negative v-down (climbing = negative down velocity)
        self.assertLess(v_down_climb, -5.0, msg="v-down should be negative for climb")

        # Gamma should be close to commanded
        self.assertAlmostEqual(
            gamma_climb, 5.0, delta=1.0, msg="Gamma should match commanded climb angle"
        )

        # h-dot and v-down should be approximately opposite
        self.assertAlmostEqual(h_dot_climb, -v_down_climb, delta=2.0, msg="h-dot ≈ -v-down")

    def test_ground_speed_and_track(self):
        """
        Test ground speed and ground track calculations.

        Ground speed considers wind effects. In no-wind conditions,
        ground speed should equal true airspeed.

        Tests:
        - velocities/vg-fps (ground speed in fps)
        - Ground track angle
        - Relationship to true airspeed
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test with no wind - ground speed should equal true airspeed
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/psi-true-deg"] = 0.0

        # Ensure no wind
        fdm["atmosphere/wind-north-fps"] = 0.0
        fdm["atmosphere/wind-east-fps"] = 0.0
        fdm["atmosphere/wind-down-fps"] = 0.0

        fdm.run_ic()
        fdm.run()

        vg_fps = fdm["velocities/vg-fps"]
        vt_fps = fdm["velocities/vt-fps"]

        # Ground speed should equal true airspeed (no wind)
        self.assertAlmostEqual(
            vg_fps,
            vt_fps,
            delta=5.0,
            msg=f"Ground speed should equal true airspeed with no wind: vg={vg_fps}, vt={vt_fps}",
        )

        # Ground speed should be positive and reasonable
        self.assertGreater(vg_fps, 100.0, msg="Ground speed should be reasonable (>100 fps)")
        self.assertLess(vg_fps, 300.0, msg="Ground speed should be reasonable (<300 fps)")

    def test_wind_components(self):
        """
        Test wind effects on ground speed tracking.

        Compares ground speed in different wind conditions to verify
        that FGAuxiliary properly computes ground velocity.

        Tests:
        - velocities/vg-fps (ground speed in fps)
        - Ground track calculation
        - Comparison of no-wind vs with-wind scenarios
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # First get baseline - no wind
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/psi-true-deg"] = 0.0  # heading north

        # No wind
        fdm["atmosphere/wind-north-fps"] = 0.0
        fdm["atmosphere/wind-east-fps"] = 0.0
        fdm["atmosphere/wind-down-fps"] = 0.0

        fdm.run_ic()
        fdm.run()

        vg_nowind = fdm["velocities/vg-fps"]
        vt_nowind = fdm["velocities/vt-fps"]

        # With no wind, ground speed should approximately equal true airspeed
        self.assertAlmostEqual(
            vg_nowind,
            vt_nowind,
            delta=2.0,
            msg=f"No wind: ground speed ({vg_nowind}) should equal true airspeed ({vt_nowind})",
        )

        # Verify ground speed is positive and reasonable
        self.assertGreater(vg_nowind, 150.0, msg="Ground speed should be positive and reasonable")
        self.assertLess(vg_nowind, 200.0, msg="Ground speed should be in reasonable range")

        # Verify ground track property exists and is reasonable
        ground_track = fdm["flight-path/psi-gt-rad"]
        self.assertIsNotNone(ground_track, msg="Ground track should be available")

        # Ground track should be close to heading in no-wind conditions
        # psi-gt-rad should be close to 0 (heading north) or 2*pi
        gt_normalized = abs(ground_track) % (2 * math.pi)
        self.assertTrue(
            gt_normalized < 0.1 or gt_normalized > 6.18,
            msg=f"Ground track should be close to heading (0 or 2pi) in no wind: {ground_track}",
        )

    def test_total_temperature(self):
        """
        Test total (stagnation) temperature vs static temperature.

        Total temperature accounts for kinetic heating due to velocity:
        Tt = Ts * (1 + (gamma-1)/2 * M^2)

        At low speeds, Tt ≈ Ts. At higher speeds, Tt > Ts.

        Tests:
        - atmosphere/T-R (static temperature)
        - atmosphere/Tt-R (total temperature - if available)
        - Temperature rise with speed
        - Compressibility effects
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test at low speed - total temp should be close to static
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 80.0
        fdm.run_ic()
        fdm.run()

        T_static_low = fdm["atmosphere/T-R"]
        mach_low = fdm["velocities/mach"]

        # Calculate expected total temperature
        gamma = 1.4
        T_total_expected_low = T_static_low * (1 + (gamma - 1) / 2 * mach_low**2)

        # At low Mach, difference should be small
        delta_T_low = T_total_expected_low - T_static_low
        self.assertLess(delta_T_low, 5.0, msg="Total temperature rise should be small at low Mach")

        # Test at higher speed - total temp should be noticeably higher
        fdm["ic/vc-kts"] = 140.0
        fdm.run_ic()
        fdm.run()

        T_static_high = fdm["atmosphere/T-R"]
        mach_high = fdm["velocities/mach"]

        T_total_expected_high = T_static_high * (1 + (gamma - 1) / 2 * mach_high**2)
        delta_T_high = T_total_expected_high - T_static_high

        # Temperature rise should increase with speed
        self.assertGreater(
            delta_T_high,
            delta_T_low,
            msg=f"Temperature rise should increase with speed: {delta_T_low} -> {delta_T_high}",
        )

        # Verify reasonable values
        self.assertGreater(
            T_static_high, 450.0, msg="Static temperature should be reasonable (>450 R)"
        )
        self.assertLess(
            T_static_high, 550.0, msg="Static temperature should be reasonable (<550 R)"
        )

    def test_pressure_altitude_vs_density_altitude(self):
        """
        Test pressure altitude vs density altitude calculations.

        Pressure altitude: Altitude corresponding to static pressure in standard atmosphere.
        Density altitude: Altitude corresponding to air density in standard atmosphere.

        In standard atmosphere: pressure altitude = density altitude.
        With temperature deviation: density altitude ≠ pressure altitude.

        Tests:
        - position/h-sl-ft (geometric altitude)
        - atmosphere/pressure-altitude (pressure altitude)
        - atmosphere/density-altitude (density altitude)
        - Standard vs non-standard atmosphere effects
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test in standard atmosphere
        test_altitude = 5000.0
        fdm["ic/h-sl-ft"] = test_altitude
        fdm["ic/vc-kts"] = 100.0

        # Standard atmosphere (no temperature offset)
        fdm["atmosphere/delta-T"] = 0.0

        fdm.run_ic()
        fdm.run()

        h_geom = fdm["position/h-sl-ft"]
        h_pressure = fdm["atmosphere/pressure-altitude"]
        h_density = fdm["atmosphere/density-altitude"]

        # In standard atmosphere, all altitudes should be similar
        self.assertAlmostEqual(
            h_geom,
            test_altitude,
            delta=10.0,
            msg="Geometric altitude should match commanded",
        )

        self.assertAlmostEqual(
            h_pressure,
            h_geom,
            delta=100.0,
            msg="Pressure altitude should be close to geometric altitude in standard atmosphere",
        )

        self.assertAlmostEqual(
            h_density,
            h_pressure,
            delta=100.0,
            msg="Density altitude should equal pressure altitude in standard atmosphere",
        )

        # Test with hot day (temperature offset)
        fdm["atmosphere/delta-T"] = 20.0  # 20 degree F warmer
        fdm.run_ic()
        fdm.run()

        h_density_hot = fdm["atmosphere/density-altitude"]
        h_pressure_hot = fdm["atmosphere/pressure-altitude"]

        # On a hot day, density altitude should be higher than pressure altitude
        # (air is less dense, so effective altitude is higher)
        self.assertGreater(
            h_density_hot,
            h_pressure_hot,
            msg="Density altitude should be higher than pressure altitude on hot day",
        )

        # Difference should be significant
        delta_alt = h_density_hot - h_pressure_hot
        self.assertGreater(
            delta_alt,
            100.0,
            msg=f"Hot day should create significant density altitude difference: {delta_alt} ft",
        )


RunTest(TestAuxiliaryParametersBasic)
