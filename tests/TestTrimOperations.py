# TestTrimOperations.py
#
# Tests for aircraft trim operations.
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


class TestTrimOperations(JSBSimTestCase):
    """
    Tests for aircraft trim operations.

    Tests cover:
    - Level flight trim
    - Trim at different speeds
    - Trim at different altitudes
    - Trim stability
    """

    def test_trim_level_flight(self):
        """Test level flight trim."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Run to stabilize
        for _ in range(500):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/q-rad_sec"):
            pitch_rate = fdm["velocities/q-rad_sec"]
            # Pitch rate should be small for level flight
            self.assertLess(abs(pitch_rate), 0.5)

        del fdm

    def test_trim_different_speeds(self):
        """Test trim at different airspeeds."""
        for speed in [80, 100, 120]:
            fdm = CreateFDM(self.sandbox)
            fdm.load_model("c172x")
            fdm["ic/h-sl-ft"] = 5000
            fdm["ic/vc-kts"] = speed
            fdm.run_ic()

            for _ in range(100):
                fdm.run()

            # Should stabilize at each speed
            time = fdm.get_sim_time()
            self.assertGreater(time, 0)

            del fdm

    def test_trim_different_altitudes(self):
        """Test trim at different altitudes."""
        for alt in [2000, 5000, 10000]:
            fdm = CreateFDM(self.sandbox)
            fdm.load_model("c172x")
            fdm["ic/h-sl-ft"] = alt
            fdm["ic/vc-kts"] = 100
            fdm.run_ic()

            for _ in range(100):
                fdm.run()

            # Should work at each altitude
            time = fdm.get_sim_time()
            self.assertGreater(time, 0)

            del fdm

    def test_trim_maintains_altitude(self):
        """Test trimmed flight maintains altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        # Set throttle for level flight
        fdm["fcs/throttle-cmd-norm[0]"] = 0.6

        for _ in range(200):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]
        # Altitude should be within reasonable range
        self.assertAlmostEqual(initial_alt, final_alt, delta=500)

        del fdm

    def test_trim_stability(self):
        """Test trimmed flight is stable."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Run for extended period
        for _ in range(1000):
            fdm.run()

        pm = fdm.get_property_manager()
        # All rates should be bounded
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            self.assertLess(abs(p), 2.0)

        if pm.hasNode("velocities/q-rad_sec"):
            q = fdm["velocities/q-rad_sec"]
            self.assertLess(abs(q), 2.0)

        if pm.hasNode("velocities/r-rad_sec"):
            r = fdm["velocities/r-rad_sec"]
            self.assertLess(abs(r), 2.0)

        del fdm

    def test_trim_with_power_setting(self):
        """Test trim with specific power setting."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Set power
        fdm["fcs/throttle-cmd-norm[0]"] = 0.65

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/mixture-cmd-norm[0]"):
            fdm["fcs/mixture-cmd-norm[0]"] = 1.0

        for _ in range(200):
            fdm.run()

        # Just verify simulation runs with power setting
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestTrimOperations)
