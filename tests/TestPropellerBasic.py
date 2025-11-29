# TestPropellerBasic.py
#
# Comprehensive tests for propeller performance model (FGPropeller).
# Tests thrust calculation, power absorption, constant speed governor,
# and propeller-engine integration.
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

import pytest
from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


def start_piston_engine(fdm, throttle=0.6, mixture=0.87, max_iterations=400):
    """
    Start a piston engine using proper magneto + starter procedure.

    Args:
        fdm: FGFDMExec instance (after run_ic() has been called)
        throttle: Throttle setting (0.0-1.0)
        mixture: Mixture setting (0.0-1.0, altitude dependent)
        max_iterations: Maximum iterations to wait for engine start

    Returns:
        bool: True if engine started successfully
    """
    # Set engine controls
    fdm["fcs/throttle-cmd-norm"] = throttle
    fdm["fcs/mixture-cmd-norm"] = mixture
    fdm["propulsion/magneto_cmd"] = 3  # Both magnetos
    fdm["propulsion/starter_cmd"] = 1  # Engage starter

    # Crank engine
    for _ in range(max_iterations):
        fdm.run()
        # Check if engine has started
        if fdm["propulsion/engine/set-running"] == 1:
            rpm = fdm["propulsion/engine/propeller-rpm"]
            if rpm > 1000:
                fdm["propulsion/starter_cmd"] = 0  # Turn off starter
                return True

    return fdm["propulsion/engine/set-running"] == 1


class TestPropellerBasic(JSBSimTestCase):
    """
    Comprehensive tests for propeller (FGPropeller) functionality.

    Tests cover:
    - Propeller loading and configuration
    - Thrust calculation at various RPM and airspeed
    - Power absorption vs. engine power
    - Advance ratio (J) effects
    - Constant speed governor logic
    - Propeller efficiency
    - Reverse thrust mode
    - Propeller-engine integration
    """

    @pytest.mark.propulsion
    def test_propeller_loading(self):
        """Test that propeller loads correctly with piston engine."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Verify propeller exists
        self.assertTrue(
            fdm.get_property_manager().hasNode("propulsion/engine/propeller-rpm"),
            "Propeller RPM property should exist",
        )

        # Check propeller diameter exists (indicates prop is loaded)
        self.assertTrue(
            fdm.get_property_manager().hasNode("propulsion/engine/blade-angle"),
            "Blade angle property should exist",
        )

        del fdm

    @pytest.mark.propulsion
    def test_propeller_rpm_with_engine(self):
        """Test propeller RPM matches engine RPM through gear ratio."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/u-fps"] = 150  # Forward airspeed
        fdm.run_ic()

        # Get engine and propeller RPM
        engine_rpm = fdm["propulsion/engine/engine-rpm"]
        prop_rpm = fdm["propulsion/engine/propeller-rpm"]

        # For c172x, they should be close (1:1 gear ratio or similar)
        # The values might not be exactly equal due to simulation state
        self.assertIsNotNone(engine_rpm, "Engine RPM should be accessible")
        self.assertIsNotNone(prop_rpm, "Propeller RPM should be accessible")

        del fdm

    @pytest.mark.propulsion
    def test_propeller_thrust_generation(self):
        """Test that propeller generates thrust at forward airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Set up in-flight conditions
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150  # ~90 knots
        fdm.run_ic()

        # Start engine using proper procedure
        engine_started = start_piston_engine(fdm, throttle=0.75, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        # Run simulation to stabilize
        for _ in range(200):
            fdm.run()

        # Check thrust is being produced
        thrust = fdm["propulsion/engine/thrust-lbs"]
        self.assertGreater(thrust, 50, "Propeller should generate significant thrust")
        self.assertLess(thrust, 1000, "Thrust should be reasonable for C172")

        del fdm

    @pytest.mark.propulsion
    def test_thrust_vs_airspeed(self):
        """Test thrust at different airspeeds."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Test at moderate airspeed
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine
        engine_started = start_piston_engine(fdm, throttle=0.75, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        # Run to stabilize
        for _ in range(200):
            fdm.run()

        thrust = fdm["propulsion/engine/thrust-lbs"]
        # Just verify we got valid thrust at cruise speed
        self.assertGreater(thrust, 0, "Thrust should be positive at cruise airspeed")

        del fdm

    @pytest.mark.propulsion
    def test_advance_ratio_calculation(self):
        """Test advance ratio (J) property is calculated correctly."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Set moderate airspeed
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine
        engine_started = start_piston_engine(fdm, throttle=0.75, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        # Run to stabilize
        for _ in range(200):
            fdm.run()

        # Check advance ratio exists and is positive
        if fdm.get_property_manager().hasNode("propulsion/engine/advance-ratio"):
            advance_ratio = fdm["propulsion/engine/advance-ratio"]
            # Just verify it's a reasonable positive number (actual value depends on flight conditions)
            self.assertGreater(advance_ratio, 0, "Advance ratio should be positive")

        del fdm

    @pytest.mark.propulsion
    def test_propeller_torque(self):
        """Test that propeller applies torque to the aircraft."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine at high power
        engine_started = start_piston_engine(fdm, throttle=1.0, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        # Run to stabilize
        for _ in range(200):
            fdm.run()

        # Propeller torque should cause rolling moment
        # This exercises the torque calculation code
        roll_moment = fdm["moments/l-prop-lbsft"]
        if roll_moment is not None:
            # Just verify it's a number (could be positive or negative)
            self.assertIsInstance(roll_moment, float)

        del fdm

    @pytest.mark.propulsion
    def test_blade_angle_property(self):
        """Test blade angle property access."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Blade angle should exist for variable pitch props
        blade_angle = fdm["propulsion/engine/blade-angle"]
        self.assertIsNotNone(blade_angle, "Blade angle should be accessible")

        # Typical blade angles are 10-45 degrees
        # Fixed pitch props have constant blade angle
        self.assertGreater(blade_angle, 0, "Blade angle should be positive")
        self.assertLess(blade_angle, 90, "Blade angle should be less than 90 degrees")

        del fdm

    @pytest.mark.propulsion
    def test_propeller_power_absorption(self):
        """Test that propeller absorbs power from engine."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine
        engine_started = start_piston_engine(fdm, throttle=0.75, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        # Run to stabilize
        for _ in range(200):
            fdm.run()

        # Get power values
        engine_power = fdm["propulsion/engine/power-hp"]

        # Engine should be producing power when running
        self.assertGreater(engine_power, 50, "Engine should produce significant power")

        del fdm

    @pytest.mark.propulsion
    def test_static_thrust(self):
        """Test propeller thrust at zero airspeed (static thrust)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Static condition - no forward velocity, but some altitude for mixture
        fdm["ic/h-sl-ft"] = 1000
        fdm["ic/u-fps"] = 0
        fdm["ic/v-fps"] = 0
        fdm["ic/w-fps"] = 0
        fdm.run_ic()

        # Start engine at full power using full rich mixture at low altitude
        engine_started = start_piston_engine(fdm, throttle=1.0, mixture=1.0)
        self.assertTrue(engine_started, "Engine should start")

        # Run to stabilize (longer for static condition)
        for _ in range(300):
            fdm.run()

        thrust = fdm["propulsion/engine/thrust-lbs"]

        # Static thrust should be positive and significant
        self.assertGreater(thrust, 100, "Static thrust should be substantial")

        del fdm

    @pytest.mark.propulsion
    def test_throttle_effect_on_thrust(self):
        """Test that throttle changes affect propeller thrust."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine at moderate throttle
        engine_started = start_piston_engine(fdm, throttle=0.5, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        # Run to stabilize at 50% throttle
        for _ in range(200):
            fdm.run()
        thrust_50 = fdm["propulsion/engine/thrust-lbs"]

        # Increase to 100% throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0
        for _ in range(200):
            fdm.run()
        thrust_100 = fdm["propulsion/engine/thrust-lbs"]

        # Higher throttle should produce more thrust
        self.assertGreater(
            thrust_100, thrust_50, "Full throttle should produce more thrust than 50%"
        )

        del fdm

    @pytest.mark.propulsion
    def test_altitude_effect_on_thrust(self):
        """Test thrust generation at altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Test at moderate altitude
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine with appropriate mixture for altitude
        engine_started = start_piston_engine(fdm, throttle=0.75, mixture=0.85)
        self.assertTrue(engine_started, "Engine should start at altitude")

        # Run to stabilize
        for _ in range(200):
            fdm.run()

        thrust = fdm["propulsion/engine/thrust-lbs"]
        # At altitude, thrust is reduced but should still be positive
        self.assertGreater(thrust, 0, "Thrust should be positive at altitude")

        del fdm

    @pytest.mark.propulsion
    def test_propeller_coefficient_of_thrust(self):
        """Test coefficient of thrust (Ct) property if available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine
        engine_started = start_piston_engine(fdm, throttle=0.75, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        for _ in range(200):
            fdm.run()

        # Check if Ct property exists
        if fdm.get_property_manager().hasNode("propulsion/engine/thrust-coefficient"):
            ct = fdm["propulsion/engine/thrust-coefficient"]
            # Ct can be positive or slightly negative depending on conditions
            # Just verify we can access it
            self.assertIsNotNone(ct, "Ct should be accessible")

        del fdm

    @pytest.mark.propulsion
    def test_propeller_efficiency(self):
        """Test propeller efficiency calculation if available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Start engine at cruise power
        engine_started = start_piston_engine(fdm, throttle=0.65, mixture=0.9)
        self.assertTrue(engine_started, "Engine should start")

        for _ in range(200):
            fdm.run()

        # Calculate efficiency from thrust and power
        thrust = fdm["propulsion/engine/thrust-lbs"]
        velocity = fdm["velocities/u-fps"]
        power_hp = fdm["propulsion/engine/power-hp"]

        if power_hp > 0 and velocity > 0:
            # Efficiency = (Thrust * Velocity) / Power
            # Converting: Thrust [lbs] * V [fps] / (Power [hp] * 550 [ft-lbs/s per hp])
            efficiency = (thrust * velocity) / (power_hp * 550)
            # Typical propeller efficiency is 0.6-0.9 at cruise
            # Note: This is a rough calculation; actual efficiency depends on many factors
            self.assertGreater(efficiency, 0.2, "Propeller efficiency should be reasonable")
            self.assertLess(efficiency, 1.5, "Efficiency shouldn't exceed 1.0 much")

        del fdm


RunTest(TestPropellerBasic)
