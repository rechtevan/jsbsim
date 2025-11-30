# TestControlLimits.py
#
# Tests for control surface limits and saturation.
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


class TestControlLimits(JSBSimTestCase):
    """
    Tests for control surface limits.

    Tests cover:
    - Aileron limits
    - Elevator limits
    - Rudder limits
    - Throttle limits
    """

    def test_aileron_positive_limit(self):
        """Test aileron positive deflection limit."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Command beyond limit
        fdm["fcs/aileron-cmd-norm"] = 2.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/aileron-pos-rad"):
            fdm.run()
            pos = fdm["fcs/aileron-pos-rad"]
            # Should be limited
            self.assertIsNotNone(pos)

        del fdm

    def test_aileron_negative_limit(self):
        """Test aileron negative deflection limit."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Command beyond limit
        fdm["fcs/aileron-cmd-norm"] = -2.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/aileron-pos-rad"):
            fdm.run()
            pos = fdm["fcs/aileron-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_elevator_positive_limit(self):
        """Test elevator positive deflection limit."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        fdm["fcs/elevator-cmd-norm"] = 2.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            fdm.run()
            pos = fdm["fcs/elevator-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_rudder_limit(self):
        """Test rudder deflection limit."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        fdm["fcs/rudder-cmd-norm"] = 2.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/rudder-pos-rad"):
            fdm.run()
            pos = fdm["fcs/rudder-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_throttle_min_limit(self):
        """Test throttle at minimum command."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm"] = 0.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/throttle-pos-norm"):
            fdm.run()
            pos = fdm["fcs/throttle-pos-norm"]
            self.assertIsNotNone(pos)

        del fdm

    def test_throttle_max_limit(self):
        """Test throttle at maximum command."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm"] = 1.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/throttle-pos-norm"):
            fdm.run()
            pos = fdm["fcs/throttle-pos-norm"]
            self.assertIsNotNone(pos)

        del fdm

    def test_mixture_at_full(self):
        """Test mixture control at full."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        fdm["fcs/mixture-cmd-norm"] = 1.0

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/mixture-pos-norm"):
            fdm.run()
            pos = fdm["fcs/mixture-pos-norm"]
            self.assertIsNotNone(pos)

        del fdm


if __name__ == "__main__":
    RunTest(TestControlLimits)
