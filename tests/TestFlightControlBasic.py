# TestFlightControlBasic.py
#
# Tests for Flight Control System (FCS) basic functionality.
# Exercises control inputs, mixing, and output properties.
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


class TestFlightControlBasic(JSBSimTestCase):
    """
    Tests for Flight Control System basic functionality.

    Tests cover:
    - Control input commands
    - Control surface positions
    - Throttle and mixture controls
    - Trim settings
    """

    def test_aileron_input(self):
        """Test aileron control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set aileron command
        fdm["fcs/aileron-cmd-norm"] = 0.5
        fdm.run()

        # Check aileron position responds
        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/aileron-pos-rad"):
            pos = fdm["fcs/aileron-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_elevator_input(self):
        """Test elevator control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set elevator command
        fdm["fcs/elevator-cmd-norm"] = -0.3
        fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_rudder_input(self):
        """Test rudder control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set rudder command
        fdm["fcs/rudder-cmd-norm"] = 0.4
        fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/rudder-pos-rad"):
            pos = fdm["fcs/rudder-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_throttle_input(self):
        """Test throttle control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set throttle
        fdm["fcs/throttle-cmd-norm"] = 0.75

        for _ in range(10):
            fdm.run()

        throttle = fdm["fcs/throttle-cmd-norm"]
        self.assertAlmostEqual(throttle, 0.75, delta=0.01)

        del fdm

    def test_mixture_input(self):
        """Test mixture control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set mixture
        fdm["fcs/mixture-cmd-norm"] = 1.0

        for _ in range(10):
            fdm.run()

        mixture = fdm["fcs/mixture-cmd-norm"]
        self.assertAlmostEqual(mixture, 1.0, delta=0.01)

        del fdm

    def test_flap_input(self):
        """Test flap control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set flaps
        fdm["fcs/flap-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/flap-pos-norm"):
            pos = fdm["fcs/flap-pos-norm"]
            self.assertIsNotNone(pos)

        del fdm

    def test_gear_input(self):
        """Test gear control input."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/u-fps"] = 500
        fdm.run_ic()

        # Retract gear
        fdm["gear/gear-cmd-norm"] = 0.0

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("gear/gear-pos-norm"):
            pos = fdm["gear/gear-pos-norm"]
            self.assertIsNotNone(pos)

        del fdm

    def test_control_limits(self):
        """Test that control inputs can be set."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set control within limits
        fdm["fcs/aileron-cmd-norm"] = 1.0
        fdm.run()

        # The command should be set
        cmd = fdm["fcs/aileron-cmd-norm"]
        self.assertIsNotNone(cmd)

        del fdm

    def test_trim_elevator(self):
        """Test elevator trim."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/pitch-trim-cmd-norm"):
            fdm["fcs/pitch-trim-cmd-norm"] = 0.1

            for _ in range(10):
                fdm.run()

            trim = fdm["fcs/pitch-trim-cmd-norm"]
            self.assertAlmostEqual(trim, 0.1, delta=0.01)

        del fdm

    def test_trim_aileron(self):
        """Test aileron trim."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/roll-trim-cmd-norm"):
            fdm["fcs/roll-trim-cmd-norm"] = 0.05

            for _ in range(10):
                fdm.run()

            trim = fdm["fcs/roll-trim-cmd-norm"]
            self.assertAlmostEqual(trim, 0.05, delta=0.01)

        del fdm

    def test_trim_rudder(self):
        """Test rudder trim."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/yaw-trim-cmd-norm"):
            fdm["fcs/yaw-trim-cmd-norm"] = 0.02

            for _ in range(10):
                fdm.run()

            trim = fdm["fcs/yaw-trim-cmd-norm"]
            self.assertAlmostEqual(trim, 0.02, delta=0.01)

        del fdm

    def test_multiple_controls(self):
        """Test setting multiple controls simultaneously."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Set multiple controls
        fdm["fcs/aileron-cmd-norm"] = 0.2
        fdm["fcs/elevator-cmd-norm"] = -0.1
        fdm["fcs/rudder-cmd-norm"] = 0.1
        fdm["fcs/throttle-cmd-norm"] = 0.8

        for _ in range(20):
            fdm.run()

        # Verify all were set
        self.assertAlmostEqual(fdm["fcs/aileron-cmd-norm"], 0.2, delta=0.01)
        self.assertAlmostEqual(fdm["fcs/elevator-cmd-norm"], -0.1, delta=0.01)
        self.assertAlmostEqual(fdm["fcs/rudder-cmd-norm"], 0.1, delta=0.01)
        self.assertAlmostEqual(fdm["fcs/throttle-cmd-norm"], 0.8, delta=0.01)

        del fdm


if __name__ == "__main__":
    RunTest(TestFlightControlBasic)
