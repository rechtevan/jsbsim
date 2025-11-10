# test_jsbsim_utils.py
#
# Unit tests for JSBSim test utilities and autopilot infrastructure.
#
# Copyright (c) 2025 Booz Allen Hamilton Inc.
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

import os
import sys

import pytest

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from JSBSim_utils import (  # noqa: E402
    AltitudeHoldController,
    HeadingHoldController,
    SandBox,
    SimplePIDController,
    SpeedHoldController,
    TrimAircraft,
)


class TestSimplePIDController:
    """Unit tests for SimplePIDController class."""

    def test_pid_initialization(self):
        """Test PID controller initializes with correct parameters."""
        pid = SimplePIDController(kp=1.0, ki=0.1, kd=0.01, output_min=-1.0, output_max=1.0)

        assert pid.kp == 1.0
        assert pid.ki == 0.1
        assert pid.kd == 0.01
        assert pid.output_min == -1.0
        assert pid.output_max == 1.0
        assert pid.integral == 0.0
        assert pid.last_error == 0.0
        assert pid.last_time is None

    def test_pid_proportional_term(self):
        """Test proportional term produces correct output."""
        # Pure P controller (ki=0, kd=0)
        pid = SimplePIDController(kp=2.0, ki=0.0, kd=0.0, output_min=-10.0, output_max=10.0)

        # First update initializes, returns 0
        output = pid.update(error=0.0, current_time=0.0)
        assert output == 0.0

        # Second update produces P term
        output = pid.update(error=1.0, current_time=1.0)
        assert output == 2.0  # kp * error = 2.0 * 1.0

        output = pid.update(error=-0.5, current_time=2.0)
        assert output == -1.0  # kp * error = 2.0 * -0.5

    def test_pid_integral_term(self):
        """Test integral term accumulates error over time."""
        # Pure I controller (kp=0, kd=0)
        pid = SimplePIDController(kp=0.0, ki=0.5, kd=0.0, output_min=-10.0, output_max=10.0)

        pid.update(error=0.0, current_time=0.0)  # Initialize

        # Error of 1.0 for 1 second
        output = pid.update(error=1.0, current_time=1.0)
        assert output == 0.5  # ki * (error * dt) = 0.5 * (1.0 * 1.0)

        # Error of 1.0 for another 1 second (integral accumulates)
        output = pid.update(error=1.0, current_time=2.0)
        assert output == 1.0  # ki * accumulated = 0.5 * (1.0 + 1.0)

    def test_pid_derivative_term(self):
        """Test derivative term responds to rate of change."""
        # Pure D controller (kp=0, ki=0)
        pid = SimplePIDController(kp=0.0, ki=0.0, kd=2.0, output_min=-10.0, output_max=10.0)

        pid.update(error=0.0, current_time=0.0)  # Initialize

        # Error changes from 0 to 1 in 1 second
        output = pid.update(error=1.0, current_time=1.0)
        assert output == 2.0  # kd * (error_change / dt) = 2.0 * (1.0 / 1.0)

        # Error stays at 1 (no change)
        output = pid.update(error=1.0, current_time=2.0)
        assert output == 0.0  # kd * (0 / 1.0)

    def test_pid_output_clamping(self):
        """Test output is clamped to min/max limits."""
        pid = SimplePIDController(kp=10.0, ki=0.0, kd=0.0, output_min=-2.0, output_max=2.0)

        pid.update(error=0.0, current_time=0.0)

        # Large positive error should clamp to max
        output = pid.update(error=1.0, current_time=1.0)
        assert output == 2.0  # Clamped to output_max

        # Large negative error should clamp to min
        output = pid.update(error=-1.0, current_time=2.0)
        assert output == -2.0  # Clamped to output_min

    def test_pid_anti_windup(self):
        """Test integral windup protection when output saturates."""
        pid = SimplePIDController(kp=0.0, ki=1.0, kd=0.0, output_min=-1.0, output_max=1.0)

        pid.update(error=0.0, current_time=0.0)

        # Accumulate integral beyond saturation
        pid.update(error=5.0, current_time=1.0)  # integral = 5, output = 1 (saturated)

        # Integral should be reset due to anti-windup
        assert pid.integral == 0.0

    def test_pid_reset(self):
        """Test reset() clears controller state."""
        pid = SimplePIDController(kp=1.0, ki=0.1, kd=0.01, output_min=-10.0, output_max=10.0)

        # Run controller to build up state (avoid saturation to preserve integral)
        pid.update(error=0.0, current_time=0.0)
        pid.update(error=0.5, current_time=1.0)
        pid.update(error=0.5, current_time=2.0)

        # Verify state exists
        assert pid.last_error == 0.5
        assert pid.last_time == 2.0

        # Reset
        pid.reset()

        # Verify state cleared
        assert pid.integral == 0.0
        assert pid.last_error == 0.0
        assert pid.last_time is None

    def test_pid_zero_dt(self):
        """Test controller handles zero time delta gracefully."""
        pid = SimplePIDController(kp=1.0, ki=1.0, kd=1.0, output_min=-10.0, output_max=10.0)

        pid.update(error=0.0, current_time=1.0)

        # Same time (dt = 0)
        output = pid.update(error=1.0, current_time=1.0)
        assert output == 0.0  # Should return 0 for zero dt

    def test_pid_combined_terms(self):
        """Test PID controller with all terms active."""
        pid = SimplePIDController(kp=1.0, ki=0.5, kd=0.1, output_min=-10.0, output_max=10.0)

        pid.update(error=0.0, current_time=0.0)

        # Error = 2.0 for 1 second
        # P = 1.0 * 2.0 = 2.0
        # I = 0.5 * (2.0 * 1.0) = 1.0
        # D = 0.1 * (2.0 / 1.0) = 0.2
        # Total = 3.2
        output = pid.update(error=2.0, current_time=1.0)
        assert abs(output - 3.2) < 0.001


class TestHeadingHoldController:
    """Unit tests for HeadingHoldController function."""

    def test_heading_hold_basic(self):
        """Test heading hold produces aileron command."""

        # Mock FDM
        class MockFDM:
            def __init__(self):
                self.props = {"attitude/psi-deg": 0.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Create explicit PID controller to reuse across calls
        pid = SimplePIDController(kp=0.01, ki=0.001, kd=0.005, output_min=-0.3, output_max=0.3)

        # Target heading 90°, current heading 0°
        # Error = 90°
        # First call initializes PID
        HeadingHoldController(fdm, target_heading_deg=90.0, pid_controller=pid)
        # Second call produces output
        aileron_cmd, rudder_cmd = HeadingHoldController(
            fdm, target_heading_deg=90.0, pid_controller=pid
        )

        # Should command positive aileron (right turn)
        assert aileron_cmd > 0.0
        assert rudder_cmd == aileron_cmd * 0.5  # Coordinated rudder

    def test_heading_hold_wrap_around_positive(self):
        """Test heading controller handles 360° wrap-around correctly."""

        class MockFDM:
            def __init__(self):
                self.props = {"attitude/psi-deg": 350.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Create explicit PID controller to reuse across calls
        pid = SimplePIDController(kp=0.01, ki=0.001, kd=0.005, output_min=-0.3, output_max=0.3)

        # Target heading 10°, current heading 350°
        # Shortest path: +20° (not -340°)
        HeadingHoldController(fdm, target_heading_deg=10.0, pid_controller=pid)  # Initialize
        aileron_cmd, rudder_cmd = HeadingHoldController(
            fdm, target_heading_deg=10.0, pid_controller=pid
        )

        # Should command positive aileron (right turn for shortest path)
        assert aileron_cmd > 0.0

    def test_heading_hold_wrap_around_negative(self):
        """Test heading controller handles negative wrap-around."""

        class MockFDM:
            def __init__(self):
                self.props = {"attitude/psi-deg": 10.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Create explicit PID controller to reuse across calls
        pid = SimplePIDController(kp=0.01, ki=0.001, kd=0.005, output_min=-0.3, output_max=0.3)

        # Target heading 350°, current heading 10°
        # Shortest path: -20° (not +340°)
        HeadingHoldController(fdm, target_heading_deg=350.0, pid_controller=pid)  # Initialize
        aileron_cmd, rudder_cmd = HeadingHoldController(
            fdm, target_heading_deg=350.0, pid_controller=pid
        )

        # Should command negative aileron (left turn for shortest path)
        assert aileron_cmd < 0.0

    def test_heading_hold_coordinated_rudder(self):
        """Test rudder coordination with aileron."""

        class MockFDM:
            def __init__(self):
                self.props = {"attitude/psi-deg": 0.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        aileron_cmd, rudder_cmd = HeadingHoldController(fdm, target_heading_deg=45.0)

        # Rudder should be 50% of aileron
        assert rudder_cmd == aileron_cmd * 0.5


class TestSpeedHoldController:
    """Unit tests for SpeedHoldController function."""

    def test_speed_hold_basic(self):
        """Test speed hold produces throttle command."""

        class MockFDM:
            def __init__(self):
                self.props = {"velocities/vc-kts": 80.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Create explicit PID controller to reuse across calls
        pid = SimplePIDController(kp=0.02, ki=0.005, kd=0.01, output_min=0.0, output_max=1.0)

        # Target speed 100 kts, current speed 80 kts
        # Error = +20 kts (need to speed up)
        SpeedHoldController(fdm, target_speed_kts=100.0, pid_controller=pid)  # Initialize
        throttle_cmd = SpeedHoldController(fdm, target_speed_kts=100.0, pid_controller=pid)

        # Should command positive throttle increase
        assert throttle_cmd > 0.0

    def test_speed_hold_reduce_speed(self):
        """Test speed hold reduces throttle when too fast."""

        class MockFDM:
            def __init__(self):
                self.props = {"velocities/vc-kts": 120.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Target speed 100 kts, current speed 120 kts
        # Error = -20 kts (need to slow down)
        throttle_cmd = SpeedHoldController(fdm, target_speed_kts=100.0)

        # Throttle command should be reduced (could be 0 or negative based on PID state)
        # After first call, should be 0 (initialization)
        # After second call with accumulated error, should reduce
        throttle_cmd = SpeedHoldController(fdm, target_speed_kts=100.0)
        assert throttle_cmd < 1.0  # Should be trying to reduce throttle

    def test_speed_hold_throttle_limits(self):
        """Test throttle is clamped to 0.0-1.0 range."""

        class MockFDM:
            def __init__(self):
                self.props = {"velocities/vc-kts": 50.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Very large speed error
        for _ in range(10):
            throttle_cmd = SpeedHoldController(fdm, target_speed_kts=200.0)

        # Should clamp to max throttle
        assert throttle_cmd <= 1.0
        assert throttle_cmd >= 0.0


class TestSandBox:
    """Unit tests for SandBox utility class."""

    def test_sandbox_call_method(self):
        """Test SandBox.__call__() returns correct relative path."""
        sandbox = SandBox()

        try:
            # Test __call__ method with file path
            rel_path = sandbox("test_file.txt")

            # Should return a relative path (not absolute)
            assert not os.path.isabs(rel_path)

            # Should contain the filename
            assert "test_file.txt" in rel_path

            # Test with subdirectory
            rel_path_subdir = sandbox("subdir", "test_file.txt")
            assert "subdir" in rel_path_subdir
            assert "test_file.txt" in rel_path_subdir
        finally:
            sandbox.erase()

    def test_sandbox_exists_method(self):
        """Test SandBox.exists() uses __call__() correctly."""
        sandbox = SandBox()

        try:
            # Create a test file in the sandbox
            test_file = "test_exists.txt"
            full_path = os.path.join(sandbox._tmpdir, test_file)

            # File doesn't exist yet
            assert not sandbox.exists(test_file)

            # Create the file
            with open(full_path, "w") as f:
                f.write("test")

            # Now it should exist
            assert sandbox.exists(test_file)
        finally:
            sandbox.erase()


class TestAltitudeHoldEdgeCases:
    """Unit tests for AltitudeHoldController edge cases."""

    def test_altitude_hold_default_pid(self):
        """Test AltitudeHoldController creates default PID when None."""

        class MockFDM:
            def __init__(self):
                self.props = {"position/h-sl-ft": 1000.0}
                self.sim_time = 0.0

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def get_sim_time(self):
                self.sim_time += 1.0
                return self.sim_time

        fdm = MockFDM()

        # Call without PID controller (should create default)
        # First call initializes
        AltitudeHoldController(fdm, target_altitude_ft=2000.0, pid_controller=None)

        # Second call should produce output
        elevator_cmd = AltitudeHoldController(fdm, target_altitude_ft=2000.0, pid_controller=None)

        # Should return a reasonable elevator command (not None, not NaN)
        assert elevator_cmd is not None
        assert not (elevator_cmd != elevator_cmd)  # Check for NaN

        # Should be within reasonable elevator limits
        assert -1.0 <= elevator_cmd <= 1.0


class TestTrimAircraftRetry:
    """Unit tests for TrimAircraft retry logic."""

    def test_trim_retry_on_failure(self):
        """Test TrimAircraft retries after initial failure."""
        # Import TrimFailureError from jsbsim if available
        try:
            from jsbsim import TrimFailureError
        except ImportError:
            pytest.skip("jsbsim module not available for TrimFailureError")

        class MockFDM:
            def __init__(self):
                self.props = {
                    "fcs/throttle-cmd-norm": 0.0,
                    "simulation/trim-completed": 0,
                }
                self.run_count = 0
                self.trim_attempts = 0

            def __setitem__(self, key, value):
                self.props[key] = value
                # Simulate trim behavior
                if key == "simulation/do_simple_trim" and value == 1:
                    self.trim_attempts += 1
                    # First attempt fails, second succeeds
                    if self.trim_attempts == 1:
                        raise TrimFailureError("First trim failed")
                    else:
                        self.props["simulation/trim-completed"] = 1

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def run(self):
                self.run_count += 1
                return True

        fdm = MockFDM()

        # Call TrimAircraft - should handle failure and retry
        result = TrimAircraft(fdm, throttle_guess=0.6)

        # Verify trim succeeded after retry
        assert result is True

        # Verify two trim attempts were made
        assert fdm.trim_attempts == 2

        # Verify additional run() calls between attempts (should be 20)
        # Initial: 10 runs, after first failure: 20 runs
        # Total should be 30+ runs
        assert fdm.run_count >= 30

    def test_trim_fails_after_retry(self):
        """Test TrimAircraft returns False when both attempts fail."""
        try:
            from jsbsim import TrimFailureError
        except ImportError:
            pytest.skip("jsbsim module not available for TrimFailureError")

        class MockFDM:
            def __init__(self):
                self.props = {"fcs/throttle-cmd-norm": 0.0}
                self.trim_attempts = 0

            def __setitem__(self, key, value):
                self.props[key] = value
                if key == "simulation/do_simple_trim" and value == 1:
                    self.trim_attempts += 1
                    # Both attempts fail
                    raise TrimFailureError("Trim failed")

            def __getitem__(self, key):
                return self.props.get(key, 0.0)

            def run(self):
                return True

        fdm = MockFDM()

        # Call TrimAircraft - should fail both times
        result = TrimAircraft(fdm, throttle_guess=0.6)

        # Verify trim failed
        assert result is False

        # Verify two attempts were made
        assert fdm.trim_attempts == 2


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
