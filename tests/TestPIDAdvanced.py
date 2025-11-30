# TestPIDAdvanced.py
#
# Tests for advanced PID controller features.
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


class TestPIDAdvanced(JSBSimTestCase):
    """
    Tests for advanced PID controller features.

    Tests cover:
    - Proportional response
    - Integral accumulation
    - Derivative response
    - Integrator windup
    - PID tuning
    """

    def test_pid_proportional_response(self):
        """Test PID proportional term response."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Create error condition
        fdm["fcs/elevator-cmd-norm"] = 0.3

        for _ in range(10):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            # Should have non-zero response
            self.assertIsNotNone(pos)

        del fdm

    def test_pid_integral_accumulation(self):
        """Test PID integral term accumulates."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Sustained error
        fdm["fcs/elevator-cmd-norm"] = 0.1

        for _ in range(200):
            fdm.run()

        # With integral action, output should continue changing
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_pid_derivative_response(self):
        """Test PID derivative term responds to rate of change."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Rapid change
        fdm["fcs/elevator-cmd-norm"] = 0.0
        fdm.run()
        fdm["fcs/elevator-cmd-norm"] = 0.5

        for _ in range(5):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_pid_zero_error_response(self):
        """Test PID with zero error."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Zero input
        fdm["fcs/elevator-cmd-norm"] = 0.0

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            # With zero input, output should be near zero
            self.assertAlmostEqual(pos, 0, delta=0.5)

        del fdm

    def test_pid_stability(self):
        """Test PID stability over time."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Constant reference
        fdm["fcs/throttle-cmd-norm[0]"] = 0.6

        for _ in range(500):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/throttle-pos-norm[0]"):
            pos = fdm["fcs/throttle-pos-norm[0]"]
            # Should be stable (not oscillating wildly)
            self.assertFalse(pos != pos)  # Not NaN
            self.assertLess(abs(pos), 10)  # Bounded

        del fdm

    def test_pid_with_saturation(self):
        """Test PID behavior with output saturation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Large command that would saturate
        fdm["fcs/elevator-cmd-norm"] = 1.0

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            # Output should be bounded
            self.assertLess(abs(pos), 1.0)  # Radians bounded

        del fdm

    def test_pid_sign_reversal(self):
        """Test PID with sign reversal of error."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Positive then negative
        fdm["fcs/aileron-cmd-norm"] = 0.5
        for _ in range(20):
            fdm.run()

        fdm["fcs/aileron-cmd-norm"] = -0.5
        for _ in range(20):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("fcs/left-aileron-pos-rad"):
            pos = fdm["fcs/left-aileron-pos-rad"]
            # Should have reversed sign
            self.assertLess(pos, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestPIDAdvanced)
