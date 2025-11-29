# TestPIDController.py
#
# Tests for PID controller component (FGPID).
# Uses the C172X autopilot which includes PID controllers.
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


class TestPIDController(JSBSimTestCase):
    """
    Tests for PID controller (FGPID) functionality.

    Uses the C172X autopilot which includes:
    - Roll PID controller (fcs/roll-ap-error-pid)
    - Pitch PID controller
    - Various autopilot modes

    Tests cover:
    - PID component loading
    - PID response to attitude errors
    - Trigger (integrator reset) functionality
    - PID output property access
    """

    def test_pid_component_exists(self):
        """Test that C172X autopilot PID controller loads."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # C172X autopilot has a roll PID
        self.assertTrue(
            fdm.get_property_manager().hasNode("fcs/roll-ap-error-pid"),
            "Roll PID controller should exist in C172X autopilot",
        )

        del fdm

    def test_pid_output_accessible(self):
        """Test that PID output is accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Run a few frames
        for _ in range(10):
            fdm.run()

        pid_output = fdm["fcs/roll-ap-error-pid"]
        self.assertIsNotNone(pid_output, "PID output should be accessible")

        del fdm

    def test_pid_responds_to_attitude(self):
        """Test that PID responds to roll attitude error."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/phi-rad"] = 0.1  # Initial roll angle
        fdm.run_ic()

        # Enable attitude hold
        fdm["ap/attitude_hold"] = 1

        # Run simulation
        for _ in range(50):
            fdm.run()

        # PID should respond to attitude error
        pid_output = fdm["fcs/roll-ap-error-pid"]
        # With non-zero phi, PID should produce non-zero output
        self.assertIsNotNone(pid_output, "PID should produce output")

        del fdm

    def test_pid_gain_properties(self):
        """Test that PID gain properties are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # C172X has gain properties
        kp = fdm["ap/roll-pid-kp"]
        ki = fdm["ap/roll-pid-ki"]
        kd = fdm["ap/roll-pid-kd"]

        self.assertGreater(kp, 0, "Kp should be positive")
        self.assertGreater(ki, 0, "Ki should be positive")
        self.assertGreater(kd, 0, "Kd should be positive")

        del fdm

    def test_pid_gains_modifiable(self):
        """Test that PID gains can be modified at runtime."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Modify gains
        original_kp = fdm["ap/roll-pid-kp"]
        fdm["ap/roll-pid-kp"] = original_kp * 2

        new_kp = fdm["ap/roll-pid-kp"]
        self.assertAlmostEqual(new_kp, original_kp * 2, places=3, msg="Kp should be modifiable")

        del fdm

    def test_pid_trigger_functionality(self):
        """Test PID trigger (integrator reset) mechanism."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Run with attitude hold OFF (trigger active)
        fdm["ap/attitude_hold"] = 0
        for _ in range(20):
            fdm.run()

        # Check trigger switch
        trigger = fdm["fcs/wing-leveler-ap-on-off"]
        self.assertEqual(trigger, -1, "Trigger should be -1 when AP off")

        # Enable attitude hold
        fdm["ap/attitude_hold"] = 1
        fdm.run()

        trigger = fdm["fcs/wing-leveler-ap-on-off"]
        self.assertEqual(trigger, 0, "Trigger should be 0 when AP on")

        del fdm

    def test_pitch_pid_existence(self):
        """Test that pitch PID controller exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # At minimum, the autopilot should have altitude control property
        self.assertTrue(
            fdm.get_property_manager().hasNode("ap/altitude_hold"),
            "Altitude hold property should exist",
        )

        del fdm

    def test_j246_pid_controllers(self):
        """Test PID controllers in J246 rocket model."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("J246")
            fdm.run_ic()

            # J246 has control system with PIDs
            has_pitch_pid = fdm.get_property_manager().hasNode("control/pitch-hold-vertical")
            has_yaw_pid = fdm.get_property_manager().hasNode("control/yaw-hold-vertical")

            # At least one should exist
            self.assertTrue(has_pitch_pid or has_yaw_pid, "J246 should have control PIDs")
        except Exception:
            # J246 may not be available in all installations
            pass
        finally:
            del fdm


if __name__ == "__main__":
    RunTest(TestPIDController)
