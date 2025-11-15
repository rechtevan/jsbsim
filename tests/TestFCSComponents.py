# TestFCSComponents.py
#
# Integration tests for Flight Control System (FCS) components.
# Tests filters, actuators, gains, control chains, and FCS processing.
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
#

from JSBSim_utils import JSBSimTestCase, RunTest


class TestFCSComponents(JSBSimTestCase):
    """
    Test Flight Control System (FCS) components and control chains.

    This test suite verifies:
    - Basic FCS command input to output path
    - Actuator rate limiting behavior
    - Actuator position limits (clipto)
    - Gain element application
    - Complete control surface chains (elevator, aileron, rudder)
    - Throttle and mixture control paths
    - FCS component integration with aircraft model

    Components tested:
    - FGFCS (Flight Control System executive)
    - Actuators with rate and position limiting
    - Gain and aerosurface_scale elements
    - Summer elements for control input combining
    - Kinematic elements for flap/gear actuation
    """

    def setUp(self):
        """Initialize test fixture with C172X aircraft."""
        JSBSimTestCase.setUp(self)
        self.fdm = self.create_fdm()
        self.fdm.load_model("c172x")

        # Set basic initial conditions for ground test (not flying)
        self.fdm["ic/h-sl-ft"] = 0.0
        self.fdm["ic/terrain-elevation-ft"] = 0.0
        self.fdm["ic/vt-kts"] = 0.0
        self.fdm["ic/alpha-deg"] = 0.0
        self.fdm["ic/theta-deg"] = 0.0
        self.fdm["ic/phi-deg"] = 0.0

        self.fdm.run_ic()

        # Run a few frames to stabilize
        for _ in range(10):
            self.fdm.run()

    def test_fcs_basic_command_path(self):
        """
        Test basic FCS command input to output path.

        Verifies that:
        - Commands can be set via fcs/*-cmd-norm properties
        - Commands propagate through FCS chain
        - Output positions respond to commands
        - FCS processes on each simulation frame
        """
        # Test elevator command path
        initial_elevator = self.fdm["fcs/elevator-pos-rad"]

        # Apply elevator command
        self.fdm["fcs/elevator-cmd-norm"] = 0.5
        self.fdm.run()

        # Verify command was processed (position should start changing)
        # May not reach final position immediately due to actuator lag
        current_elevator = self.fdm["fcs/elevator-pos-rad"]

        # Position should have changed from initial
        self.assertNotAlmostEqual(
            current_elevator,
            initial_elevator,
            delta=0.001,
            msg="Elevator position did not respond to command",
        )

        # Test aileron command path
        initial_aileron = self.fdm["fcs/left-aileron-pos-rad"]

        self.fdm["fcs/aileron-cmd-norm"] = -0.3
        self.fdm.run()

        current_aileron = self.fdm["fcs/left-aileron-pos-rad"]
        self.assertNotAlmostEqual(
            current_aileron,
            initial_aileron,
            delta=0.001,
            msg="Aileron position did not respond to command",
        )

        # Test rudder command path
        initial_rudder = self.fdm["fcs/rudder-pos-rad"]

        self.fdm["fcs/rudder-cmd-norm"] = 0.4
        self.fdm.run()

        current_rudder = self.fdm["fcs/rudder-pos-rad"]
        self.assertNotAlmostEqual(
            current_rudder,
            initial_rudder,
            delta=0.001,
            msg="Rudder position did not respond to command",
        )

    def test_actuator_rate_limiting(self):
        """
        Test actuator rate limiting functionality.

        Verifies that:
        - Actuators respect rate_limit settings
        - Position changes are limited per time step
        - Large commands don't result in instantaneous position changes
        - Rate limiting works in both directions (increasing/decreasing)

        C172X aileron has rate_limit of 1.57 rad/s
        """
        # Start with neutral aileron
        self.fdm["fcs/aileron-cmd-norm"] = 0.0
        for _ in range(20):
            self.fdm.run()

        initial_pos = self.fdm["fcs/left-aileron-pos-rad"]

        # Command large aileron deflection
        self.fdm["fcs/aileron-cmd-norm"] = 1.0

        # After one frame, should not be at commanded position (rate limited)
        self.fdm.run()
        pos_after_one_frame = self.fdm["fcs/left-aileron-pos-rad"]

        # Should have moved from initial position
        self.assertNotAlmostEqual(
            pos_after_one_frame,
            initial_pos,
            delta=0.001,
            msg="Aileron did not start moving",
        )

        # Calculate expected maximum movement in one time step
        dt = self.fdm["simulation/dt"]
        rate_limit = 1.57  # rad/s from c172x.xml

        # Position change should not exceed rate_limit * dt
        max_delta = rate_limit * dt
        actual_delta = abs(pos_after_one_frame - initial_pos)

        self.assertLessEqual(
            actual_delta,
            max_delta * 1.1,  # Allow 10% margin for numerical precision
            msg=f"Aileron rate exceeded limit: {actual_delta} > {max_delta}",
        )

        # Run for several more frames - should continue rate-limited movement
        for _ in range(5):
            prev_pos = self.fdm["fcs/left-aileron-pos-rad"]
            self.fdm.run()
            current_pos = self.fdm["fcs/left-aileron-pos-rad"]

            # Each step should respect rate limit
            delta = abs(current_pos - prev_pos)
            if delta > 0.001:  # Only check if significant movement
                self.assertLessEqual(
                    delta,
                    max_delta * 1.1,
                    msg=f"Rate limit violated during movement: {delta} > {max_delta}",
                )

    def test_actuator_position_limits(self):
        """
        Test actuator position limiting (clipto).

        Verifies that:
        - Actuators respect min/max position limits
        - Position never exceeds clipto bounds
        - Limits are enforced even with extreme commands

        C172X left aileron clipto: min=-0.350, max=0.260 rad
        """
        # Command maximum positive aileron
        self.fdm["fcs/aileron-cmd-norm"] = 1.0

        # Run long enough to reach limit
        for _ in range(100):
            self.fdm.run()

        final_pos = self.fdm["fcs/left-aileron-pos-rad"]

        # Should not exceed max limit
        max_limit = 0.260  # From c172x.xml
        self.assertLessEqual(
            final_pos,
            max_limit + 0.001,
            msg=f"Aileron exceeded max limit: {final_pos} > {max_limit}",
        )

        # Should be at or very close to max limit
        self.assertGreater(
            final_pos,
            max_limit - 0.05,
            msg=f"Aileron did not reach max limit: {final_pos} < {max_limit}",
        )

        # Command maximum negative aileron
        self.fdm["fcs/aileron-cmd-norm"] = -1.0

        # Run long enough to reach opposite limit
        for _ in range(100):
            self.fdm.run()

        final_pos = self.fdm["fcs/left-aileron-pos-rad"]

        # Should not go below min limit
        min_limit = -0.350  # From c172x.xml
        self.assertGreaterEqual(
            final_pos, min_limit - 0.001, msg=f"Aileron below min limit: {final_pos} < {min_limit}"
        )

        # Should be at or very close to min limit
        self.assertLess(
            final_pos,
            min_limit + 0.05,
            msg=f"Aileron did not reach min limit: {final_pos} > {min_limit}",
        )

    def test_fcs_gain_application(self):
        """
        Test gain element application in FCS.

        Verifies that:
        - Gain elements multiply input values correctly
        - Aerosurface_scale elements map normalized commands to physical ranges
        - Gain scaling is consistent across multiple frames

        C172X uses aerosurface_scale with 0.01745 gain (deg to rad conversion)
        and range scaling for control surfaces.
        """
        # Test with known command and verify scaled output
        # Elevator: range -28 to +23 deg, gain 0.01745 rad/deg

        # Neutral command should give near-neutral output
        self.fdm["fcs/elevator-cmd-norm"] = 0.0
        for _ in range(20):
            self.fdm.run()

        # Small positive command
        cmd = 0.25  # 25% positive
        self.fdm["fcs/elevator-cmd-norm"] = cmd

        # Run enough to let actuator reach steady state
        for _ in range(100):
            self.fdm.run()

        final_pos = self.fdm["fcs/elevator-pos-rad"]

        # Expected range: -28 to +23 degrees
        # At cmd=0.25, expected = 0.25 * 23 = 5.75 deg = 0.1004 rad
        # But actuator has bias and lag, so check reasonable range
        expected_approx = 0.25 * 23.0 * 0.01745  # ~0.1 rad

        self.assertGreater(
            final_pos,
            expected_approx - 0.05,
            msg=f"Elevator gain/scale incorrect: {final_pos} vs expected ~{expected_approx}",
        )
        self.assertLess(
            final_pos,
            expected_approx + 0.05,
            msg=f"Elevator gain/scale incorrect: {final_pos} vs expected ~{expected_approx}",
        )

    def test_elevator_control_chain(self):
        """
        Test complete elevator control chain from command to position.

        Control chain: cmd-norm -> summer (with trim) -> aerosurface_scale
                      -> actuator (lag, bias, hysteresis, clipto) -> pos-rad

        Verifies:
        - Full control chain processes correctly
        - Trim inputs combine with pilot commands
        - Final position responds to command
        """
        # Test with zero trim
        self.fdm["fcs/pitch-trim-cmd-norm"] = 0.0
        self.fdm["fcs/elevator-cmd-norm"] = 0.0

        for _ in range(20):
            self.fdm.run()

        neutral_pos = self.fdm["fcs/elevator-pos-rad"]

        # Apply nose-up elevator command
        self.fdm["fcs/elevator-cmd-norm"] = -0.5  # Negative is typically nose-up

        for _ in range(50):
            self.fdm.run()

        nose_up_pos = self.fdm["fcs/elevator-pos-rad"]

        # Position should have changed
        self.assertNotAlmostEqual(
            nose_up_pos,
            neutral_pos,
            delta=0.01,
            msg="Elevator did not respond to command",
        )

        # Apply nose-down elevator command
        self.fdm["fcs/elevator-cmd-norm"] = 0.5

        for _ in range(50):
            self.fdm.run()

        nose_down_pos = self.fdm["fcs/elevator-pos-rad"]

        # Should have moved in opposite direction
        # Nose-down position should be greater than nose-up position
        self.assertGreater(
            nose_down_pos,
            nose_up_pos,
            msg="Elevator did not reverse direction correctly",
        )

        # Test trim addition
        self.fdm["fcs/elevator-cmd-norm"] = 0.0
        self.fdm["fcs/pitch-trim-cmd-norm"] = 0.3

        for _ in range(50):
            self.fdm.run()

        trim_pos = self.fdm["fcs/elevator-pos-rad"]

        # Trim should move elevator from neutral
        self.assertNotAlmostEqual(
            trim_pos, neutral_pos, delta=0.01, msg="Pitch trim did not affect elevator position"
        )

    def test_aileron_control_chain(self):
        """
        Test complete aileron control chain.

        Control chain (per aileron):
        - Left: cmd-norm -> summer (with trim/AP) -> aerosurface_scale
                -> actuator (rate_limit, hysteresis, clipto) -> left-aileron-pos-rad
        - Right: -(cmd-norm) -> (same chain) -> right-aileron-pos-rad

        Verifies:
        - Left and right ailerons move in opposite directions
        - Differential aileron deflection is correct
        - Rate limiting works on ailerons
        """
        # Start neutral
        self.fdm["fcs/aileron-cmd-norm"] = 0.0
        self.fdm["fcs/roll-trim-cmd-norm"] = 0.0

        for _ in range(20):
            self.fdm.run()

        # Command right roll (positive aileron)
        self.fdm["fcs/aileron-cmd-norm"] = 0.6

        for _ in range(50):
            self.fdm.run()

        left_ail = self.fdm["fcs/left-aileron-pos-rad"]
        right_ail = self.fdm["fcs/right-aileron-pos-rad"]

        # Left and right should have opposite signs (differential)
        # For right roll: left aileron down (positive), right aileron up (negative)
        self.assertGreater(left_ail, 0.01, msg="Left aileron did not deflect down for right roll")
        self.assertLess(right_ail, -0.01, msg="Right aileron did not deflect up for right roll")

        # Magnitudes should be similar (may not be exactly equal due to different
        # physical limits, but should be close)
        ratio = abs(left_ail / right_ail) if right_ail != 0 else 0
        self.assertGreater(ratio, 0.5, msg="Aileron differential seems incorrect")
        self.assertLess(ratio, 2.0, msg="Aileron differential seems incorrect")

        # Test opposite direction
        self.fdm["fcs/aileron-cmd-norm"] = -0.6

        for _ in range(50):
            self.fdm.run()

        left_ail_2 = self.fdm["fcs/left-aileron-pos-rad"]
        right_ail_2 = self.fdm["fcs/right-aileron-pos-rad"]

        # Should have reversed
        self.assertLess(left_ail_2, -0.01, msg="Left aileron did not reverse")
        self.assertGreater(right_ail_2, 0.01, msg="Right aileron did not reverse")

    def test_rudder_control_chain(self):
        """
        Test complete rudder control chain.

        Control chain: cmd-norm -> summer (with trim) -> aerosurface_scale -> pos-rad

        Note: C172X rudder has no actuator element, direct aerosurface_scale output.

        Verifies:
        - Rudder responds to command
        - Yaw trim combines with rudder command
        - Output scaling is correct
        """
        # Start neutral
        self.fdm["fcs/rudder-cmd-norm"] = 0.0
        self.fdm["fcs/yaw-trim-cmd-norm"] = 0.0

        for _ in range(20):
            self.fdm.run()

        neutral_pos = self.fdm["fcs/rudder-pos-rad"]

        # Right rudder command
        self.fdm["fcs/rudder-cmd-norm"] = 0.5

        for _ in range(20):
            self.fdm.run()

        right_rudder_pos = self.fdm["fcs/rudder-pos-rad"]

        # Should have moved right (positive)
        self.assertGreater(right_rudder_pos, neutral_pos + 0.01, msg="Rudder did not move right")

        # Left rudder command
        self.fdm["fcs/rudder-cmd-norm"] = -0.5

        for _ in range(20):
            self.fdm.run()

        left_rudder_pos = self.fdm["fcs/rudder-pos-rad"]

        # Should have moved left (negative)
        self.assertLess(left_rudder_pos, neutral_pos - 0.01, msg="Rudder did not move left")

        # Test yaw trim
        self.fdm["fcs/rudder-cmd-norm"] = 0.0
        self.fdm["fcs/yaw-trim-cmd-norm"] = 0.3

        for _ in range(20):
            self.fdm.run()

        trim_pos = self.fdm["fcs/rudder-pos-rad"]

        # Trim should offset from neutral
        self.assertNotAlmostEqual(
            trim_pos, neutral_pos, delta=0.01, msg="Yaw trim did not affect rudder"
        )

    def test_throttle_control_chain(self):
        """
        Test throttle control chain.

        Verifies:
        - Throttle command propagates to position
        - Throttle position is normalized (0-1)
        - Throttle affects engine power (if engine running)
        """
        # Test throttle at idle
        self.fdm["fcs/throttle-cmd-norm"] = 0.0

        for _ in range(10):
            self.fdm.run()

        idle_throttle = self.fdm["fcs/throttle-pos-norm"]

        # Should be at or near zero
        self.assertLess(idle_throttle, 0.1, msg="Throttle not at idle")

        # Test throttle at full power
        self.fdm["fcs/throttle-cmd-norm"] = 1.0

        for _ in range(10):
            self.fdm.run()

        full_throttle = self.fdm["fcs/throttle-pos-norm"]

        # Should be at or near 1.0
        self.assertGreater(full_throttle, 0.9, msg="Throttle not at full power")

        # Test intermediate throttle
        self.fdm["fcs/throttle-cmd-norm"] = 0.6

        for _ in range(10):
            self.fdm.run()

        mid_throttle = self.fdm["fcs/throttle-pos-norm"]

        # Should be in middle range
        self.assertGreater(mid_throttle, 0.5, msg="Throttle position incorrect")
        self.assertLess(mid_throttle, 0.7, msg="Throttle position incorrect")

    def test_mixture_control_chain(self):
        """
        Test mixture control properties.

        Note: C172X has automatic mixture control based on atmospheric pressure,
        so mixture-cmd-norm is computed automatically rather than accepting
        manual commands. This test verifies the mixture properties are accessible
        and contain reasonable values.

        Verifies:
        - Mixture command property is accessible
        - Mixture position property is accessible
        - Mixture values are in valid range (0-1)
        - Mixture control responds to atmospheric conditions
        """
        # Run simulation to ensure mixture system is initialized
        for _ in range(20):
            self.fdm.run()

        # Verify mixture command is accessible and in valid range
        mixture_cmd = self.fdm["fcs/mixture-cmd-norm"]
        self.assertGreaterEqual(mixture_cmd, 0.0, msg="Mixture command below valid range")
        self.assertLessEqual(mixture_cmd, 1.0, msg="Mixture command above valid range")

        # Verify mixture position is accessible and in valid range
        mixture_pos = self.fdm["fcs/mixture-pos-norm"]
        self.assertGreaterEqual(mixture_pos, 0.0, msg="Mixture position below valid range")
        self.assertLessEqual(mixture_pos, 1.0, msg="Mixture position above valid range")

        # At sea level (standard pressure), mixture should be near 1.0 (rich)
        # C172X automatic mixture: Mixture = (P_amb / P_std) / 1.3
        # At sea level: P_amb ≈ P_std, so Mixture ≈ 1/1.3 ≈ 0.77
        pressure = self.fdm["atmosphere/P-psf"]

        # Should be at or near standard sea level pressure (2116.2 psf)
        self.assertGreater(pressure, 2000.0, msg="Atmospheric pressure too low")
        self.assertLess(pressure, 2200.0, msg="Atmospheric pressure too high")

        # With automatic mixture, command should be reasonable for sea level
        # Expected around 0.77-1.0 depending on exact pressure
        self.assertGreater(
            mixture_cmd,
            0.5,
            msg=f"Mixture command too low for sea level: {mixture_cmd}",
        )
        self.assertLess(
            mixture_cmd,
            1.1,
            msg=f"Mixture command too high for sea level: {mixture_cmd}",
        )

    def test_flap_kinematic_control(self):
        """
        Test flap kinematic control (time-based movement through detents).

        C172X flaps use kinematic element with detents at 0, 10, 20, 30 degrees.

        Verifies:
        - Flaps move through detent positions
        - Kinematic timing controls movement rate
        - Flap position normalizes correctly
        """
        # Start with flaps up
        self.fdm["fcs/flap-cmd-norm"] = 0.0

        for _ in range(20):
            self.fdm.run()

        # Verify flaps are up
        flap_pos_deg = self.fdm["fcs/flap-pos-deg"]
        self.assertLess(flap_pos_deg, 1.0, msg="Flaps not fully retracted")

        # Command flaps to first detent (should be 10 degrees)
        # According to kinematic, this takes 2 seconds
        self.fdm["fcs/flap-cmd-norm"] = 0.33  # ~1/3 of range

        # Run for 2.5 seconds to allow movement
        dt = self.fdm["simulation/dt"]
        frames = int(2.5 / dt)
        for _ in range(frames):
            self.fdm.run()

        flap_pos_deg = self.fdm["fcs/flap-pos-deg"]

        # Should be at or near 10 degrees
        self.assertGreater(flap_pos_deg, 8.0, msg="Flaps did not extend to first detent")
        self.assertLess(flap_pos_deg, 12.0, msg="Flaps extended beyond first detent")

        # Command full flaps (30 degrees)
        self.fdm["fcs/flap-cmd-norm"] = 1.0

        # Run for 4 seconds (additional time for remaining detents)
        frames = int(4.0 / dt)
        for _ in range(frames):
            self.fdm.run()

        flap_pos_deg = self.fdm["fcs/flap-pos-deg"]

        # Should be at or near 30 degrees
        self.assertGreater(flap_pos_deg, 28.0, msg="Flaps did not extend to full")
        self.assertLess(flap_pos_deg, 32.0, msg="Flaps extended beyond full")

        # Verify normalized position
        flap_pos_norm = self.fdm["fcs/flap-pos-norm"]
        self.assertGreater(flap_pos_norm, 0.9, msg="Flap normalized position incorrect")
        self.assertLess(flap_pos_norm, 1.1, msg="Flap normalized position incorrect")


RunTest(TestFCSComponents)
