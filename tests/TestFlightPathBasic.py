# TestFlightPathBasic.py
#
# Tests for flight path and trajectory properties.
# Exercises flight path angle, track angle, and navigation.
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


class TestFlightPathBasic(JSBSimTestCase):
    """
    Tests for flight path and trajectory properties.

    Tests cover:
    - Flight path angle (gamma)
    - Ground track angle
    - True airspeed
    - Ground speed
    - Vertical speed (climb/descent rate)
    """

    def test_flight_path_angle_level(self):
        """Test flight path angle in level flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 0.0  # Level flight
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        # Flight path angle should be near zero
        if fdm.get_property_manager().hasNode("flight-path/gamma-deg"):
            gamma = fdm["flight-path/gamma-deg"]
            self.assertAlmostEqual(
                gamma, 0.0, delta=5.0, msg="Gamma should be near 0 in level flight"
            )

        del fdm

    def test_flight_path_angle_climb(self):
        """Test flight path angle during climb."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 5.0  # Climbing
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Check altitude is increasing (confirms climb)
        hdot = fdm["velocities/h-dot-fps"]
        self.assertGreater(hdot, 0, "Should be climbing (positive hdot)")

        del fdm

    def test_true_airspeed(self):
        """Test true airspeed calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100  # Calibrated airspeed
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # True airspeed (higher than CAS at altitude)
        vt = fdm["velocities/vt-fps"]
        self.assertGreater(vt, 0, "True airspeed should be positive")

        del fdm

    def test_ground_speed(self):
        """Test ground speed calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Ground speed
        if fdm.get_property_manager().hasNode("velocities/vg-fps"):
            vg = fdm["velocities/vg-fps"]
            self.assertGreater(vg, 0, "Ground speed should be positive")

        del fdm

    def test_vertical_speed_climb(self):
        """Test vertical speed during climb."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 5.0  # Climbing
        fdm.run_ic()

        for _ in range(30):
            fdm.run()

        # Vertical speed should be positive (climbing)
        hdot = fdm["velocities/h-dot-fps"]
        self.assertGreater(hdot, 0, "Hdot should be positive when climbing")

        del fdm

    def test_vertical_speed_descent(self):
        """Test vertical speed during descent."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = -5.0  # Descending
        fdm.run_ic()

        for _ in range(30):
            fdm.run()

        # Vertical speed should be negative (descending)
        hdot = fdm["velocities/h-dot-fps"]
        self.assertLess(hdot, 0, "Hdot should be negative when descending")

        del fdm

    def test_track_heading(self):
        """Test ground track/heading."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/psi-true-deg"] = 90.0  # Heading east
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Heading should be approximately east
        psi = fdm["attitude/psi-deg"]
        # Allow for some drift
        self.assertGreater(psi, 45, "Heading should be generally east")
        self.assertLess(psi, 135, "Heading should be generally east")

        del fdm

    def test_mach_number(self):
        """Test Mach number calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        mach = fdm["velocities/mach"]
        # C172 at 150 fps is well subsonic
        self.assertGreater(mach, 0, "Mach should be positive")
        self.assertLess(mach, 0.3, "C172 should be subsonic")

        del fdm

    def test_equivalent_airspeed(self):
        """Test equivalent airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        if fdm.get_property_manager().hasNode("velocities/ve-fps"):
            ve = fdm["velocities/ve-fps"]
            self.assertGreater(ve, 0, "EAS should be positive")

        del fdm

    def test_calibrated_airspeed(self):
        """Test calibrated airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        vc = fdm["velocities/vc-kts"]
        self.assertGreater(vc, 50, "CAS should be significant")
        self.assertLess(vc, 200, "CAS should be reasonable for C172")

        del fdm

    def test_indicated_airspeed(self):
        """Test indicated airspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        if fdm.get_property_manager().hasNode("velocities/vc-kts"):
            vc = fdm["velocities/vc-kts"]
            self.assertIsNotNone(vc, "Airspeed should be accessible")

        del fdm

    def test_position_change_over_time(self):
        """Test that position changes over flight time."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/psi-true-deg"] = 0.0  # North
        fdm.run_ic()

        initial_lat = fdm["position/lat-gc-deg"]

        # Fly for a while
        for _ in range(500):
            fdm.run()

        final_lat = fdm["position/lat-gc-deg"]

        # Should have moved north
        self.assertGreater(final_lat, initial_lat, "Should move north over time")

        del fdm


if __name__ == "__main__":
    RunTest(TestFlightPathBasic)
