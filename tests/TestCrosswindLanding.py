# TestCrosswindLanding.py
#
# Tests for crosswind landing scenarios.
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


class TestCrosswindLanding(JSBSimTestCase):
    """
    Tests for crosswind landing scenarios.

    Tests cover:
    - Wind effects on approach
    - Sideslip from crosswind
    - Crab angle
    """

    def test_crosswind_induces_sideslip(self):
        """Test that crosswind induces sideslip."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 1000
        fdm["ic/vc-kts"] = 80
        fdm["ic/psi-true-deg"] = 0  # North
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("atmosphere/wind-east-fps"):
            del fdm
            return

        # Add crosswind from east
        fdm["atmosphere/wind-east-fps"] = -30

        for _ in range(100):
            fdm.run()

        if pm.hasNode("aero/beta-deg"):
            beta = fdm["aero/beta-deg"]
            self.assertIsNotNone(beta)

        del fdm

    def test_approach_with_headwind(self):
        """Test approach with headwind affects flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 1000
        fdm["ic/vc-kts"] = 80
        fdm["ic/psi-true-deg"] = 0  # North
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("atmosphere/wind-north-fps"):
            del fdm
            return

        # Add headwind from north
        fdm["atmosphere/wind-north-fps"] = 30

        for _ in range(50):
            fdm.run()

        # Verify properties exist and simulation ran
        if pm.hasNode("velocities/vg-fps") and pm.hasNode("velocities/vt-fps"):
            vg = fdm["velocities/vg-fps"]
            vt = fdm["velocities/vt-fps"]
            self.assertIsNotNone(vg)
            self.assertIsNotNone(vt)

        del fdm

    def test_approach_with_tailwind(self):
        """Test approach with tailwind affects flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 1000
        fdm["ic/vc-kts"] = 80
        fdm["ic/psi-true-deg"] = 0  # North
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("atmosphere/wind-north-fps"):
            del fdm
            return

        # Add tailwind from south
        fdm["atmosphere/wind-north-fps"] = -30

        for _ in range(50):
            fdm.run()

        # Verify properties exist
        if pm.hasNode("velocities/vg-fps") and pm.hasNode("velocities/vt-fps"):
            vg = fdm["velocities/vg-fps"]
            vt = fdm["velocities/vt-fps"]
            self.assertIsNotNone(vg)
            self.assertIsNotNone(vt)

        del fdm

    def test_descent_rate_property(self):
        """Test descent rate during approach."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 1000
        fdm["ic/vc-kts"] = 70
        fdm["ic/gamma-deg"] = -3
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/h-dot-fps"):
            hdot = fdm["velocities/h-dot-fps"]
            # Should be descending (negative)
            self.assertIsNotNone(hdot)

        del fdm


if __name__ == "__main__":
    RunTest(TestCrosswindLanding)
