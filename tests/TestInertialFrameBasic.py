# TestInertialFrameBasic.py
#
# Comprehensive tests for inertial reference frame calculations (FGInertial)
# Tests gravity models, Earth rotation, reference frame transformations
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


class TestInertialFrameBasic(JSBSimTestCase):
    """
    Comprehensive tests for inertial reference frame calculations.

    Tests FGInertial model implementation including:
    - Earth reference radius calculations
    - Gravitational acceleration (WGS84 and standard models)
    - Gravity variation with latitude and altitude
    - Earth rotation rate (omega)
    - Reference frame transformations
    """

    def setUp(self, *args):
        JSBSimTestCase.setUp(self, *args)

        # WGS84 Earth parameters (from FGInertial.cpp)
        self.a_wgs84_ft = 20925646.32546  # Semimajor axis (equatorial radius)
        self.b_wgs84_ft = 20855486.5951  # Semiminor axis (polar radius)
        self.GM_wgs84 = 14.0764417572e15  # Gravitational parameter (ft^3/s^2)
        self.J2_wgs84 = 1.08262982e-03  # Second zonal harmonic
        self.omega_earth_rad_sec = 0.00007292115  # Earth rotation rate

        # Standard gravity at 45 deg latitude (includes centripetal acceleration)
        # From FGInertial.h: 9.80665 m/s^2 converted to ft/s^2
        self.g_standard_fps2 = 32.174048556430446  # 9.80665 / 0.3048

        # Conversion factors
        self.deg_to_rad = math.pi / 180.0
        self.rad_to_deg = 180.0 / math.pi

    def test_reference_radius_at_equator(self):
        """
        Test Earth reference radius at equator.

        At the equator (lat = 0 deg), sea-level radius should equal
        the WGS84 semimajor axis (equatorial radius).
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 0.0  # Equator
        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0  # Sea level
        fdm.run_ic()

        radius_ft = fdm["inertial/sea-level-radius_ft"]

        # At equator, radius should equal semimajor axis
        self.assertAlmostEqual(
            radius_ft,
            self.a_wgs84_ft,
            delta=1.0,
            msg=f"Sea level radius at equator should be ~{self.a_wgs84_ft} ft (WGS84 semimajor axis)",
        )

    def test_reference_radius_at_pole(self):
        """
        Test Earth reference radius at North Pole.

        At the pole (lat = 90 deg), sea-level radius should equal
        the WGS84 semiminor axis (polar radius).
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 90.0  # North Pole
        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0  # Sea level
        fdm.run_ic()

        radius_ft = fdm["inertial/sea-level-radius_ft"]

        # At pole, radius should equal semiminor axis
        self.assertAlmostEqual(
            radius_ft,
            self.b_wgs84_ft,
            delta=1.0,
            msg=f"Sea level radius at pole should be ~{self.b_wgs84_ft} ft (WGS84 semiminor axis)",
        )

    def test_reference_radius_varies_with_latitude(self):
        """
        Test that Earth reference radius varies smoothly with latitude.

        Radius should:
        - Be maximum at equator (semimajor axis)
        - Be minimum at poles (semiminor axis)
        - Decrease monotonically from equator to pole
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        latitudes = [0, 15, 30, 45, 60, 75, 90]
        radii = []

        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0

        for lat in latitudes:
            fdm["ic/lat-geod-deg"] = lat
            fdm.run_ic()
            radius = fdm["inertial/sea-level-radius_ft"]
            radii.append(radius)

        # Verify radius decreases monotonically from equator to pole
        for i in range(len(radii) - 1):
            self.assertGreater(
                radii[i],
                radii[i + 1],
                msg=f"Radius should decrease from lat {latitudes[i]} deg to {latitudes[i+1]} deg",
            )

        # Verify bounds
        self.assertAlmostEqual(
            radii[0], self.a_wgs84_ft, delta=1.0, msg="Radius at equator should be semimajor axis"
        )
        self.assertAlmostEqual(
            radii[-1], self.b_wgs84_ft, delta=1.0, msg="Radius at pole should be semiminor axis"
        )

    def test_gravity_at_sea_level(self):
        """
        Test gravitational acceleration at sea level.

        At sea level, gravity should be approximately 32.174 ft/s^2
        (standard gravity at 45 deg latitude).
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 45.0  # 45 deg latitude (standard)
        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0  # Sea level
        fdm.run_ic()

        g_fps2 = fdm["accelerations/gravity-ft_sec2"]

        # Gravity at 45 deg, sea level should be near standard gravity
        # JSBSim WGS84 gives ~32.228 ft/s^2 at 45 deg (includes J2 effect)
        self.assertAlmostEqual(
            g_fps2,
            self.g_standard_fps2,
            delta=0.1,  # Allow larger tolerance for WGS84 model differences
            msg=f"Gravity at sea level, 45 deg should be ~{self.g_standard_fps2} ft/s^2",
        )

    def test_gravity_decreases_with_altitude(self):
        """
        Test that gravity decreases with altitude (inverse square law).

        Gravity should decrease as altitude increases due to increasing
        distance from Earth's center.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 45.0
        fdm["ic/long-gc-deg"] = 0.0

        altitudes = [0, 10000, 30000, 50000, 100000, 200000]
        gravities = []

        for alt in altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            g = fdm["accelerations/gravity-ft_sec2"]
            gravities.append(g)

        # Verify gravity decreases monotonically with altitude
        for i in range(len(gravities) - 1):
            self.assertGreater(
                gravities[i],
                gravities[i + 1],
                msg=f"Gravity should decrease from {altitudes[i]} ft to {altitudes[i+1]} ft: "
                f"{gravities[i]} > {gravities[i+1]}",
            )

        # Verify gravity at sea level is highest
        self.assertGreater(
            gravities[0],
            gravities[-1],
            msg=f"Gravity at sea level ({gravities[0]}) should be greater than at 200,000 ft ({gravities[-1]})",
        )

        # At 200,000 ft (~61 km), gravity should be roughly 97-98% of sea level
        # r = 20,890,713 ft (sea level at 45 deg) + 200,000 ft = 21,090,713 ft
        # g_ratio = (r_sl / r_alt)^2 = (20,890,713 / 21,090,713)^2 ≈ 0.981
        expected_ratio = 0.981
        actual_ratio = gravities[-1] / gravities[0]
        self.assertAlmostEqual(
            actual_ratio,
            expected_ratio,
            delta=0.005,
            msg=f"Gravity at 200,000 ft should be ~{expected_ratio*100}% of sea level gravity",
        )

    def test_gravity_variation_with_latitude(self):
        """
        Test gravity variation with latitude.

        Due to:
        1. Earth's oblateness (J2 effect) - radius varies
        2. Centrifugal force - maximum at equator, zero at poles

        Gravity should be:
        - Minimum at equator (~32.088 ft/s^2)
        - Maximum at poles (~32.258 ft/s^2)
        - Intermediate at 45 deg (~32.174 ft/s^2, standard gravity)
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0

        # Test at equator, 45 deg, and pole
        test_latitudes = [0, 45, 90]
        gravities = []

        for lat in test_latitudes:
            fdm["ic/lat-geod-deg"] = lat
            fdm.run_ic()
            g = fdm["accelerations/gravity-ft_sec2"]
            gravities.append(g)

        g_equator = gravities[0]
        g_45deg = gravities[1]
        g_pole = gravities[2]

        # Gravity at equator should be less than at pole
        self.assertLess(
            g_equator,
            g_pole,
            msg=f"Gravity at equator ({g_equator}) should be less than at pole ({g_pole})",
        )

        # Gravity at 45 deg should be between equator and pole
        self.assertGreater(
            g_45deg,
            g_equator,
            msg=f"Gravity at 45 deg ({g_45deg}) should be greater than at equator ({g_equator})",
        )
        self.assertLess(
            g_45deg,
            g_pole,
            msg=f"Gravity at 45 deg ({g_45deg}) should be less than at pole ({g_pole})",
        )

        # Verify approximate values (JSBSim WGS84 implementation)
        # Values may differ slightly from theoretical WGS84 due to implementation details
        # Equator: ~9.780-9.820 m/s^2 = ~32.088-32.220 ft/s^2
        # Pole: ~9.832-9.850 m/s^2 = ~32.258-32.320 ft/s^2
        self.assertGreater(
            g_equator,
            32.0,
            msg="Gravity at equator should be > 32.0 ft/s^2",
        )
        self.assertLess(
            g_equator,
            32.3,
            msg="Gravity at equator should be < 32.3 ft/s^2",
        )
        self.assertGreater(
            g_pole,
            32.2,
            msg="Gravity at pole should be > 32.2 ft/s^2",
        )
        self.assertLess(g_pole, 32.4, msg="Gravity at pole should be < 32.4 ft/s^2")

    def test_gravity_model_wgs84(self):
        """
        Test that WGS84 gravity model is active by default.

        simulation/gravity-model should be 1 (gtWGS84).
        From FGInertial.h:
        - gtStandard = 0 (spherical Earth, simple 1/r^2)
        - gtWGS84 = 1 (oblate Earth with J2 correction)
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()

        gravity_model = fdm["simulation/gravity-model"]

        # Default should be WGS84 (value = 1)
        self.assertEqual(
            gravity_model,
            1,
            msg="Default gravity model should be WGS84 (gtWGS84 = 1)",
        )

    def test_gravity_wgs84_vs_standard(self):
        """
        Test difference between WGS84 and standard gravity models.

        At the equator, WGS84 (with J2) should give different gravity
        than standard spherical model.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 0.0  # Equator (maximum J2 effect)
        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0

        # Get gravity with WGS84 model (default)
        fdm["simulation/gravity-model"] = 1  # gtWGS84
        fdm.run_ic()
        g_wgs84 = fdm["accelerations/gravity-ft_sec2"]

        # Switch to standard spherical model
        fdm["simulation/gravity-model"] = 0  # gtStandard
        fdm.run_ic()
        g_standard = fdm["accelerations/gravity-ft_sec2"]

        # Models should give different results at equator
        # WGS84 accounts for Earth's oblateness, standard assumes sphere
        self.assertNotEqual(
            g_wgs84,
            g_standard,
            msg="WGS84 and standard gravity models should differ at equator",
        )

        # Difference should be small but measurable (< 0.5%)
        rel_diff = abs(g_wgs84 - g_standard) / g_standard
        self.assertLess(rel_diff, 0.005, msg="Gravity model difference should be less than 0.5%")
        self.assertGreater(
            rel_diff, 0.0001, msg="Gravity model difference should be measurable (> 0.01%)"
        )

    def test_vehicle_radius_increases_with_altitude(self):
        """
        Test that vehicle radius from Earth center increases with altitude.

        position/radius-to-vehicle-ft = sea-level-radius + altitude
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 45.0
        fdm["ic/long-gc-deg"] = 0.0

        altitudes = [0, 10000, 50000, 100000]
        radii = []

        for alt in altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()

            radius = fdm["position/radius-to-vehicle-ft"]
            radii.append(radius)

            # Verify radius increases with altitude
            sea_level_radius = fdm["inertial/sea-level-radius_ft"]
            h_sl = fdm["position/h-sl-ft"]

            # Radius should approximately equal sea-level-radius + altitude
            expected_radius = sea_level_radius + h_sl
            self.assertAlmostEqual(
                radius,
                expected_radius,
                delta=10.0,  # Allow small tolerance for coordinate conversions
                msg=f"Radius at {alt} ft should be ~{expected_radius} ft",
            )

        # Verify radii increase monotonically
        for i in range(len(radii) - 1):
            self.assertGreater(
                radii[i + 1],
                radii[i],
                msg=f"Radius should increase from {altitudes[i]} ft to {altitudes[i+1]} ft",
            )

    def test_geodetic_vs_geocentric_latitude(self):
        """
        Test difference between geodetic and geocentric latitude.

        For oblate ellipsoid (WGS84), geodetic latitude (used for maps)
        differs from geocentric latitude (angle from Earth center).

        Difference is maximum at 45 deg latitude (~0.19 deg or 11.5 arcmin).
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 45.0  # Set geodetic latitude
        fdm["ic/long-gc-deg"] = 0.0
        fdm["ic/h-sl-ft"] = 0.0
        fdm.run_ic()

        lat_geod_rad = fdm["position/lat-geod-rad"]
        lat_gc_rad = fdm["position/lat-gc-rad"]

        lat_geod_deg = lat_geod_rad * self.rad_to_deg
        lat_gc_deg = lat_gc_rad * self.rad_to_deg

        # Geodetic latitude should be what we set
        self.assertAlmostEqual(
            lat_geod_deg,
            45.0,
            places=4,
            msg="Geodetic latitude should be 45.0 deg as set",
        )

        # Geocentric latitude should be slightly less at 45 deg
        self.assertLess(
            lat_gc_deg,
            lat_geod_deg,
            msg="Geocentric latitude should be less than geodetic at 45 deg",
        )

        # Maximum difference at 45 deg is about 0.19 deg (from WGS84 theory)
        diff_deg = abs(lat_geod_deg - lat_gc_deg)
        self.assertGreater(
            diff_deg,
            0.1,
            msg="Latitude difference should be significant at 45 deg (> 0.1 deg)",
        )
        self.assertLess(
            diff_deg,
            0.2,
            msg="Latitude difference should be less than 0.2 deg at 45 deg",
        )

        # At equator and poles, geodetic = geocentric
        for test_lat in [0.0, 90.0]:
            fdm["ic/lat-geod-deg"] = test_lat
            fdm.run_ic()

            lat_geod = fdm["position/lat-geod-rad"] * self.rad_to_deg
            lat_gc = fdm["position/lat-gc-rad"] * self.rad_to_deg

            self.assertAlmostEqual(
                lat_geod,
                lat_gc,
                places=3,
                msg=f"At {test_lat} deg, geodetic and geocentric latitude should be equal",
            )

    def test_gravity_inverse_square_approximation(self):
        """
        Test that gravity follows inverse square law approximately.

        For small altitude changes, verify g(r) ≈ g0 * (r0/r)^2
        where r is distance from Earth center.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/lat-geod-deg"] = 45.0
        fdm["ic/long-gc-deg"] = 0.0

        # Get gravity and radius at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm.run_ic()
        g0 = fdm["accelerations/gravity-ft_sec2"]
        r0 = fdm["position/radius-to-vehicle-ft"]

        # Test at 50,000 ft altitude
        fdm["ic/h-sl-ft"] = 50000
        fdm.run_ic()
        g_alt = fdm["accelerations/gravity-ft_sec2"]
        r_alt = fdm["position/radius-to-vehicle-ft"]

        # Calculate expected gravity using inverse square law
        g_expected = g0 * (r0 / r_alt) ** 2

        # Actual gravity should match inverse square prediction
        self.assertAlmostEqual(
            g_alt,
            g_expected,
            delta=0.01,
            msg=f"Gravity at 50,000 ft should follow inverse square law: "
            f"expected {g_expected}, got {g_alt}",
        )

        # Verify the relationship holds for multiple altitudes
        test_altitudes = [10000, 30000, 100000]
        for alt in test_altitudes:
            fdm["ic/h-sl-ft"] = alt
            fdm.run_ic()
            g = fdm["accelerations/gravity-ft_sec2"]
            r = fdm["position/radius-to-vehicle-ft"]

            g_predicted = g0 * (r0 / r) ** 2
            rel_error = abs(g - g_predicted) / g_predicted

            self.assertLess(
                rel_error,
                0.001,  # Less than 0.1% error
                msg=f"Inverse square law should hold at {alt} ft (error < 0.1%)",
            )


RunTest(TestInertialFrameBasic)
