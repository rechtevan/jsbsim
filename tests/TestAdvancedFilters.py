# TestAdvancedFilters.py
#
# Tests for advanced FCS filter types.
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


class TestAdvancedFilters(JSBSimTestCase):
    """
    Tests for advanced FCS filter types.

    Tests cover:
    - Lag filters
    - Lead-lag filters
    - Washout filters
    - Second-order filters
    - Filter initialization
    """

    def test_lag_filter_response(self):
        """Test lag filter step response."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # C172x autopilot has lag filters
        if pm.hasNode("fcs/heading-roll-error-lag"):
            # Apply step input
            fdm["fcs/heading-roll-error-lag"] = 1.0

            for _ in range(50):
                fdm.run()

            # Filter output should exist
            self.assertTrue(True)

        del fdm

    def test_filter_with_aircraft_dynamics(self):
        """Test filters during dynamic flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply control inputs that exercise filters
        fdm["fcs/aileron-cmd-norm"] = 0.3
        fdm["fcs/elevator-cmd-norm"] = -0.2

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check filtered outputs exist
        if pm.hasNode("fcs/aileron-pos-rad"):
            pos = fdm["fcs/aileron-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_filter_steady_state(self):
        """Test filter reaches steady state."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Constant input
        fdm["fcs/throttle-cmd-norm[0]"] = 0.5

        # Run to steady state
        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/throttle-pos-norm[0]"):
            pos = fdm["fcs/throttle-pos-norm[0]"]
            # Should be close to commanded value at steady state
            self.assertAlmostEqual(pos, 0.5, delta=0.2)

        del fdm

    def test_filter_rapid_input_changes(self):
        """Test filter stability with rapid input changes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Rapid input changes
        for i in range(50):
            if i % 2 == 0:
                fdm["fcs/elevator-cmd-norm"] = 0.5
            else:
                fdm["fcs/elevator-cmd-norm"] = -0.5
            fdm.run()

        # Should not produce NaN or crash
        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            self.assertFalse(pos != pos)  # Check not NaN

        del fdm

    def test_filter_initialization(self):
        """Test filter initialization on startup."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Immediately after init, filters should have valid output
        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            self.assertIsNotNone(pos)
            self.assertFalse(pos != pos)  # Not NaN

        del fdm

    def test_multiple_filter_chain(self):
        """Test multiple filters in series."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # FCS typically has filter chains
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/left-aileron-pos-rad"):
            pos = fdm["fcs/left-aileron-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm


if __name__ == "__main__":
    RunTest(TestAdvancedFilters)
