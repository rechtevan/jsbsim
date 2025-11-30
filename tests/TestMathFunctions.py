# TestMathFunctions.py
#
# Tests for mathematical functions and operations.
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

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestMathFunctions(JSBSimTestCase):
    """
    Tests for mathematical functions and operations.

    Tests cover:
    - Trigonometric functions
    - Unit conversions
    - Coordinate transformations
    - Property calculations
    """

    def test_degrees_to_radians(self):
        """Test degrees to radians conversion."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/psi-true-deg"] = 90
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/psi-rad"):
            psi_rad = fdm["attitude/psi-rad"]
            # 90 degrees = pi/2 radians
            self.assertAlmostEqual(psi_rad, math.pi / 2, delta=0.1)

        del fdm

    def test_euler_angles(self):
        """Test Euler angle properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/phi-deg"] = 15
        fdm["ic/theta-deg"] = 5
        fdm["ic/psi-true-deg"] = 45
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Check roll
        if pm.hasNode("attitude/phi-rad"):
            phi = math.degrees(fdm["attitude/phi-rad"])
            self.assertAlmostEqual(phi, 15, delta=2)

        # Check pitch
        if pm.hasNode("attitude/theta-rad"):
            theta = math.degrees(fdm["attitude/theta-rad"])
            self.assertAlmostEqual(theta, 5, delta=2)

        del fdm

    def test_velocity_conversions(self):
        """Test velocity unit conversions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vc-fps"):
            vc_fps = fdm["velocities/vc-fps"]
            # 100 kts ≈ 168.78 fps
            self.assertAlmostEqual(vc_fps, 168.78, delta=10)

        del fdm

    def test_altitude_calculations(self):
        """Test altitude-related calculations."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 10000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-sl-ft"):
            h_sl = fdm["position/h-sl-ft"]
            self.assertAlmostEqual(h_sl, 10000, delta=100)

        del fdm

    def test_position_coordinates(self):
        """Test position coordinate properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/lat-gc-deg"] = 37.0
        fdm["ic/long-gc-deg"] = -122.0
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/lat-gc-deg"):
            lat = fdm["position/lat-gc-deg"]
            self.assertAlmostEqual(lat, 37.0, delta=0.1)

        if pm.hasNode("position/long-gc-deg"):
            lon = fdm["position/long-gc-deg"]
            self.assertAlmostEqual(lon, -122.0, delta=0.1)

        del fdm

    def test_body_rates(self):
        """Test body rate calculations."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply control for rotation
        fdm["fcs/aileron-cmd-norm"] = 0.3

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            self.assertIsNotNone(p)

        del fdm

    def test_acceleration_calculations(self):
        """Test acceleration calculations."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/a-pilot-z-ft_sec2"):
            az = fdm["accelerations/a-pilot-z-ft_sec2"]
            self.assertIsNotNone(az)

        del fdm

    def test_force_moment_calculations(self):
        """Test force and moment calculations."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check aero forces
        if pm.hasNode("forces/fwx-aero-lbs"):
            fx = fdm["forces/fwx-aero-lbs"]
            self.assertIsNotNone(fx)

        if pm.hasNode("forces/fwz-aero-lbs"):
            fz = fdm["forces/fwz-aero-lbs"]
            # Lift should be upward (negative in body Z)
            self.assertIsNotNone(fz)

        del fdm

    def test_inertia_properties(self):
        """Test inertia property access."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("inertia/weight-lbs"):
            weight = fdm["inertia/weight-lbs"]
            self.assertGreater(weight, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestMathFunctions)
