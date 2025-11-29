# TestRocketBasic.py
#
# Regression tests for the rocket engine model (FGRocket).
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

import os
import shutil

import pytest
from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestRocketBasic(JSBSimTestCase):
    """Tests for rocket engine (FGRocket) functionality."""

    def create_simple_rocket_aircraft(self, engine_file):
        """
        Create a minimal aircraft configuration with a rocket engine.

        Args:
            engine_file: Name of the rocket engine XML file to use

        Returns:
            Path to the created aircraft XML file
        """
        # Copy the engine file to sandbox
        shutil.copy(self.sandbox.path_to_jsbsim_file("engine", engine_file), self.sandbox())

        # Create a minimal aircraft with rocket propulsion
        aircraft_xml = (
            """<?xml version="1.0"?>
<fdm_config name="test_rocket" version="2.0" release="ALPHA"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sourceforge.net/JSBSim.xsd">

    <fileheader>
        <author>Test</author>
        <filecreationdate>2024-01-01</filecreationdate>
        <version>1.0</version>
        <description>Test rocket aircraft</description>
    </fileheader>

    <metrics>
        <wingarea unit="FT2">1.0</wingarea>
        <wingspan unit="FT">1.0</wingspan>
        <chord unit="FT">1.0</chord>
        <htailarea unit="FT2">0.0</htailarea>
        <htailarm unit="FT">0.0</htailarm>
        <vtailarea unit="FT2">0.0</vtailarea>
        <vtailarm unit="FT">0.0</vtailarm>
        <location name="AERORP" unit="IN">
            <x>0</x><y>0</y><z>0</z>
        </location>
        <location name="EYEPOINT" unit="IN">
            <x>0</x><y>0</y><z>0</z>
        </location>
        <location name="VRP" unit="IN">
            <x>0</x><y>0</y><z>0</z>
        </location>
    </metrics>

    <mass_balance>
        <ixx unit="SLUG*FT2">100</ixx>
        <iyy unit="SLUG*FT2">100</iyy>
        <izz unit="SLUG*FT2">100</izz>
        <emptywt unit="LBS">1000</emptywt>
        <location name="CG" unit="IN">
            <x>0</x><y>0</y><z>0</z>
        </location>
    </mass_balance>

    <ground_reactions>
        <contact type="BOGEY" name="CONTACT">
            <location unit="IN">
                <x>0</x><y>0</y><z>-12</z>
            </location>
            <static_friction>0.8</static_friction>
            <dynamic_friction>0.5</dynamic_friction>
            <rolling_friction>0.02</rolling_friction>
            <spring_coeff unit="LBS/FT">10000</spring_coeff>
            <damping_coeff unit="LBS/FT/SEC">2000</damping_coeff>
            <max_steer unit="DEG">0.0</max_steer>
            <brake_group>NONE</brake_group>
            <retractable>0</retractable>
        </contact>
    </ground_reactions>

    <propulsion>
        <engine file="%s">
            <location unit="IN">
                <x>0</x><y>0</y><z>0</z>
            </location>
            <orient unit="DEG">
                <pitch>0</pitch><roll>0</roll><yaw>0</yaw>
            </orient>
            <feed>0</feed>
            <feed>1</feed>
            <thruster file="direct">
                <location unit="IN">
                    <x>0</x><y>0</y><z>0</z>
                </location>
                <orient unit="DEG">
                    <pitch>0</pitch><roll>0</roll><yaw>0</yaw>
                </orient>
            </thruster>
        </engine>

        <tank type="FUEL" number="0">
            <location unit="IN">
                <x>0</x><y>0</y><z>0</z>
            </location>
            <capacity unit="LBS">500</capacity>
            <contents unit="LBS">500</contents>
        </tank>

        <tank type="OXIDIZER" number="1">
            <location unit="IN">
                <x>0</x><y>0</y><z>0</z>
            </location>
            <capacity unit="LBS">1000</capacity>
            <contents unit="LBS">1000</contents>
        </tank>
    </propulsion>

    <flight_control name="FCS"/>

    <aerodynamics>
        <axis name="DRAG">
            <function name="aero/coefficient/CD">
                <description>Drag</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <property>metrics/Sw-sqft</property>
                    <value>0.02</value>
                </product>
            </function>
        </axis>
    </aerodynamics>
</fdm_config>
"""
            % engine_file
        )

        # Create aircraft subdirectory in sandbox
        aircraft_dir = os.path.join(self.sandbox(), "test_rocket")
        os.makedirs(aircraft_dir, exist_ok=True)

        # Write aircraft file
        aircraft_file = os.path.join(aircraft_dir, "test_rocket.xml")
        with open(aircraft_file, "w") as f:
            f.write(aircraft_xml)

        return "test_rocket"

    @pytest.mark.propulsion
    def test_rocket_engine_loading(self):
        """Test that a rocket engine can be loaded successfully."""
        aircraft_name = self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        # Verify engine loaded
        self.assertTrue(
            fdm.get_property_manager().hasNode("propulsion/engine[0]"),
            "Rocket engine should be loaded as propulsion/engine[0]",
        )

    @pytest.mark.propulsion
    def test_rocket_thrust_at_full_throttle(self):
        """Test rocket thrust generation at full throttle."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm["ic/h-sl-ft"] = 0.0
        fdm["ic/u-fps"] = 0.0
        fdm.run_ic()

        # Set engine running and throttle to max
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["fcs/throttle-cmd-norm[0]"] = 1.0

        # Run for a few seconds
        for i in range(100):
            fdm.run()

        # Check that thrust is being produced
        thrust = fdm["propulsion/engine[0]/thrust-lbs"]
        self.assertGreater(
            thrust, 100000.0, "AJ26-33A should produce significant thrust at full throttle"
        )

    @pytest.mark.propulsion
    def test_rocket_isp_property(self):
        """Test that specific impulse (Isp) is correctly reported."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm.run_ic()

        # Check Isp property exists and has expected value
        # AJ26-33A has Isp of 331 seconds (liquid rocket)
        isp = fdm["propulsion/engine[0]/isp"]
        self.assertAlmostEqual(isp, 331.0, delta=1.0, msg="Isp should match engine specification")

    @pytest.mark.propulsion
    def test_rocket_fuel_consumption(self):
        """Test that fuel is consumed during rocket operation."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm.run_ic()

        # Record initial fuel
        initial_fuel = fdm["propulsion/tank[0]/contents-lbs"]

        # Start engine and run at full throttle
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["fcs/throttle-cmd-norm[0]"] = 1.0

        # Run for 1 second
        dt = fdm["simulation/dt"]
        steps = int(1.0 / dt)
        for i in range(steps):
            fdm.run()

        # Check that fuel decreased
        final_fuel = fdm["propulsion/tank[0]/contents-lbs"]
        self.assertLess(final_fuel, initial_fuel, "Fuel should be consumed during rocket burn")

    @pytest.mark.propulsion
    def test_rocket_oxidizer_consumption(self):
        """Test that oxidizer is consumed during rocket operation."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm.run_ic()

        # Record initial oxidizer
        initial_oxi = fdm["propulsion/tank[1]/contents-lbs"]

        # Start engine and run at full throttle
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["fcs/throttle-cmd-norm[0]"] = 1.0

        # Run for 1 second
        dt = fdm["simulation/dt"]
        steps = int(1.0 / dt)
        for i in range(steps):
            fdm.run()

        # Check that oxidizer decreased
        final_oxi = fdm["propulsion/tank[1]/contents-lbs"]
        self.assertLess(final_oxi, initial_oxi, "Oxidizer should be consumed during rocket burn")

    @pytest.mark.propulsion
    def test_rocket_throttling(self):
        """Test rocket engine throttling capability."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine[0]/set-running"] = 1

        # Test at 50% throttle
        fdm["fcs/throttle-cmd-norm[0]"] = 0.5

        # Let engine stabilize
        for i in range(50):
            fdm.run()

        thrust_50 = fdm["propulsion/engine[0]/thrust-lbs"]

        # Test at 100% throttle
        fdm["fcs/throttle-cmd-norm[0]"] = 1.0

        # Let engine stabilize
        for i in range(50):
            fdm.run()

        thrust_100 = fdm["propulsion/engine[0]/thrust-lbs"]

        # Thrust at 100% should be greater than at 50%
        self.assertGreater(
            thrust_100, thrust_50, "Thrust at 100% throttle should exceed thrust at 50%"
        )

    @pytest.mark.propulsion
    def test_rocket_total_impulse_accumulation(self):
        """Test that total impulse accumulates over time."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine[0]/set-running"] = 1
        fdm["fcs/throttle-cmd-norm[0]"] = 1.0

        # Run for a short time and check impulse increases
        for i in range(10):
            fdm.run()

        impulse_early = fdm["propulsion/engine[0]/total-impulse"]

        # Run more
        for i in range(50):
            fdm.run()

        impulse_later = fdm["propulsion/engine[0]/total-impulse"]

        # Total impulse should increase
        self.assertGreater(
            impulse_later, impulse_early, "Total impulse should accumulate over burn time"
        )

    @pytest.mark.propulsion
    def test_rocket_mixture_ratio(self):
        """Test that mixture ratio property is available and reasonable."""
        self.create_simple_rocket_aircraft("AJ26-33A.xml")

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model("test_rocket")
        fdm.run_ic()

        # Check that mixture ratio exists
        # AJ26-33A uses LOX/Kerosene with mixture ratio ~2.6
        if fdm.get_property_manager().hasNode("propulsion/engine[0]/mixture-ratio"):
            mixture_ratio = fdm["propulsion/engine[0]/mixture-ratio"]
            # Should be a positive value for oxidizer/fuel ratio
            self.assertGreater(mixture_ratio, 0.0, "Mixture ratio should be positive")


RunTest(TestRocketBasic)
