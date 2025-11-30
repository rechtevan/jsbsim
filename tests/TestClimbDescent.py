# TestClimbDescent.py
#
# Tests for climb and descent flight phases.
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


class TestClimbDescent(JSBSimTestCase):
    """
    Tests for climb and descent flight phases.

    Tests cover:
    - Climb rate
    - Descent rate
    - Flight path angle
    - Vertical speed
    """

    def test_positive_climb_rate(self):
        """Test climb produces positive altitude change."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 80
        fdm["ic/gamma-deg"] = 5  # Climbing
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        for _ in range(200):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]
        # Should be higher
        self.assertGreater(final_alt, initial_alt)

        del fdm

    def test_negative_descent_rate(self):
        """Test descent produces negative altitude change."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 80
        fdm["ic/gamma-deg"] = -5  # Descending
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        for _ in range(200):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]
        # Should be lower
        self.assertLess(final_alt, initial_alt)

        del fdm

    def test_level_flight_maintains_altitude(self):
        """Test level flight roughly maintains altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/gamma-deg"] = 0
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        for _ in range(100):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]
        # Should be roughly the same (within 500 ft for short duration)
        self.assertAlmostEqual(final_alt, initial_alt, delta=500)

        del fdm

    def test_vertical_speed_property(self):
        """Test vertical speed property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 80
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/h-dot-fps"):
            hdot = fdm["velocities/h-dot-fps"]
            self.assertIsNotNone(hdot)

        del fdm

    def test_flight_path_angle_property(self):
        """Test flight path angle property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("flight-path/gamma-deg"):
            gamma = fdm["flight-path/gamma-deg"]
            self.assertIsNotNone(gamma)

        del fdm


if __name__ == "__main__":
    RunTest(TestClimbDescent)
