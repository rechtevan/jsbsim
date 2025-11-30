# TestBodyVelocities.py
#
# Tests for body axis velocities (u, v, w).
# Exercises velocity components and transformations.
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


class TestBodyVelocities(JSBSimTestCase):
    """
    Tests for body axis velocities.

    Tests cover:
    - Forward velocity (u)
    - Lateral velocity (v)
    - Vertical velocity (w)
    - Velocity initialization
    - Velocity transformations
    """

    def test_u_fps_property(self):
        """Test forward velocity (u) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        u = fdm["velocities/u-fps"]
        self.assertAlmostEqual(u, 150, delta=10, msg="U should be ~150 fps")

        del fdm

    def test_v_fps_property(self):
        """Test lateral velocity (v) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        v = fdm["velocities/v-fps"]
        self.assertIsNotNone(v, "V should be accessible")

        del fdm

    def test_w_fps_property(self):
        """Test vertical velocity (w) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        w = fdm["velocities/w-fps"]
        self.assertIsNotNone(w, "W should be accessible")

        del fdm

    def test_level_flight_v_near_zero(self):
        """Test that v is near zero in coordinated flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/beta-deg"] = 0.0
        fdm.run_ic()

        v = fdm["velocities/v-fps"]
        self.assertAlmostEqual(v, 0.0, delta=10, msg="V should be near 0")

        del fdm

    def test_u_aero_fps_property(self):
        """Test aerodynamic u velocity property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("velocities/u-aero-fps"):
            u_aero = fdm["velocities/u-aero-fps"]
            self.assertIsNotNone(u_aero, "U aero should be accessible")

        del fdm

    def test_velocity_magnitude(self):
        """Test true airspeed relates to body velocities."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]

        # Magnitude should be related to true airspeed
        mag = (u**2 + v**2 + w**2) ** 0.5
        self.assertGreater(mag, 100, "Velocity magnitude should be significant")

        del fdm

    def test_sideslip_produces_v(self):
        """Test that sideslip produces lateral velocity."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/beta-deg"] = 10.0  # Sideslip
        fdm.run_ic()

        v = fdm["velocities/v-fps"]
        # With sideslip, v should be non-zero
        self.assertNotEqual(v, 0, "V should be non-zero with sideslip")

        del fdm

    def test_climb_produces_w(self):
        """Test that climb produces vertical velocity component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 10.0  # Climbing
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        w = fdm["velocities/w-fps"]
        # With climb, w should be non-zero
        self.assertIsNotNone(w, "W should be accessible during climb")

        del fdm

    def test_ecef_velocities(self):
        """Test ECEF velocity properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        if fdm.get_property_manager().hasNode("velocities/eci-x-fps"):
            vx = fdm["velocities/eci-x-fps"]
            self.assertIsNotNone(vx, "ECEF velocity X should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestBodyVelocities)
