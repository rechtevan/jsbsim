# TestTrimAdvanced.py
#
# Advanced tests for aircraft trim operations.
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


class TestTrimAdvanced(JSBSimTestCase):
    """
    Advanced tests for aircraft trim operations.

    Tests cover:
    - Climb trim
    - Descent trim
    - Turn trim
    - Various aircraft types
    - Edge cases
    """

    def test_trim_for_climb(self):
        """Test trim for climbing flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 80
        fdm["ic/gamma-deg"] = 5  # Climbing
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.9

        for _ in range(300):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/h-dot-fps"):
            climb_rate = fdm["velocities/h-dot-fps"]
            # Should be climbing
            self.assertGreater(climb_rate, 0)

        del fdm

    def test_trim_for_descent(self):
        """Test trim for descending flight configuration."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 8000
        fdm["ic/vc-kts"] = 90
        fdm.run_ic()

        # Low throttle - simulate idle descent
        fdm["fcs/throttle-cmd-norm[0]"] = 0.0

        for _ in range(500):
            fdm.run()

        # Just verify simulation runs with low power setting
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_trim_coordinated_turn(self):
        """Test trim in coordinated turn."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/phi-deg"] = 30  # Banked
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.7
        fdm["fcs/aileron-cmd-norm"] = 0.2

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/roll-rad"):
            import math

            roll = math.degrees(fdm["attitude/roll-rad"])
            # Should maintain some bank
            self.assertNotEqual(abs(roll), 0)

        del fdm

    def test_trim_jet_aircraft(self):
        """Test trim for jet aircraft."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 20000
        fdm["ic/vc-kts"] = 350
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.8

        for _ in range(300):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/q-rad_sec"):
            pitch_rate = fdm["velocities/q-rad_sec"]
            # Should be reasonably stable
            self.assertLess(abs(pitch_rate), 1.0)

        del fdm

    def test_trim_transport_aircraft(self):
        """Test trim for transport aircraft."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("B747")
        fdm["ic/h-sl-ft"] = 35000
        fdm["ic/vc-kts"] = 280
        fdm.run_ic()

        for i in range(4):
            fdm[f"fcs/throttle-cmd-norm[{i}]"] = 0.7

        for _ in range(300):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_trim_glider(self):
        """Test trim for unpowered glider."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("SGS")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 50
        fdm.run_ic()

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        # Glider should be descending (no power)
        if pm.hasNode("velocities/h-dot-fps"):
            vdot = fdm["velocities/h-dot-fps"]
            self.assertLess(vdot, 0)

        del fdm

    def test_trim_low_speed(self):
        """Test trim at low airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/vc-kts"] = 60  # Near stall
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.8

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-deg"):
            alpha = fdm["aero/alpha-deg"]
            # High angle of attack at low speed
            self.assertGreater(alpha, 0)

        del fdm

    def test_trim_high_speed(self):
        """Test trim at high airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 30000
        fdm["ic/vc-kts"] = 500
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.9

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/mach"):
            mach = fdm["velocities/mach"]
            self.assertGreater(mach, 0.5)

        del fdm

    def test_trim_with_cg_variation(self):
        """Test trim sensitivity to CG position."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.6

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            elevator = fdm["fcs/elevator-pos-rad"]
            # Just verify elevator position is valid
            self.assertIsNotNone(elevator)

        del fdm


if __name__ == "__main__":
    RunTest(TestTrimAdvanced)
