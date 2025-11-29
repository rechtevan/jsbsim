# TestElectricBasic.py
#
# Basic tests for electric motor model (FGElectric).
# Tests motor power output and throttle response.
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


class TestElectricBasic(JSBSimTestCase):
    """
    Basic tests for electric motor (FGElectric) functionality.

    Tests cover:
    - Electric motor loading and configuration
    - Power output at various throttle settings
    - Motor-propeller integration
    """

    def create_electric_aircraft(self):
        """
        Create a minimal aircraft configuration with an electric motor.

        Returns:
            Name of the created aircraft model
        """
        # Copy the electric engine file to sandbox
        engine_file = "electric147kW.xml"
        shutil.copy(self.sandbox.path_to_jsbsim_file("engine", engine_file), self.sandbox())

        # Copy a propeller for the motor
        prop_file = "prop_81in2v.xml"
        shutil.copy(self.sandbox.path_to_jsbsim_file("engine", prop_file), self.sandbox())

        # Create a minimal aircraft with electric propulsion
        aircraft_xml = """<?xml version="1.0"?>
<fdm_config name="test_electric" version="2.0" release="ALPHA"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sourceforge.net/JSBSim.xsd">

    <fileheader>
        <author>Test</author>
        <filecreationdate>2024-01-01</filecreationdate>
        <version>1.0</version>
        <description>Test electric aircraft</description>
    </fileheader>

    <metrics>
        <wingarea unit="FT2">174.0</wingarea>
        <wingspan unit="FT">36.0</wingspan>
        <chord unit="FT">4.9</chord>
        <htailarea unit="FT2">21.9</htailarea>
        <htailarm unit="FT">15.7</htailarm>
        <vtailarea unit="FT2">16.5</vtailarea>
        <vtailarm unit="FT">15.7</vtailarm>
        <location name="AERORP" unit="IN">
            <x>43.2</x><y>0</y><z>59.4</z>
        </location>
        <location name="EYEPOINT" unit="IN">
            <x>37</x><y>0</y><z>48</z>
        </location>
        <location name="VRP" unit="IN">
            <x>42.6</x><y>0</y><z>38.5</z>
        </location>
    </metrics>

    <mass_balance>
        <ixx unit="SLUG*FT2">948</ixx>
        <iyy unit="SLUG*FT2">1346</iyy>
        <izz unit="SLUG*FT2">1967</izz>
        <emptywt unit="LBS">1600</emptywt>
        <location name="CG" unit="IN">
            <x>41</x><y>0</y><z>36.5</z>
        </location>
    </mass_balance>

    <ground_reactions>
        <contact type="BOGEY" name="Nose Gear">
            <location unit="IN">
                <x>-6.8</x><y>0</y><z>-20</z>
            </location>
            <static_friction>0.8</static_friction>
            <dynamic_friction>0.5</dynamic_friction>
            <rolling_friction>0.02</rolling_friction>
            <spring_coeff unit="LBS/FT">1800</spring_coeff>
            <damping_coeff unit="LBS/FT/SEC">500</damping_coeff>
            <max_steer unit="DEG">10</max_steer>
            <brake_group>NONE</brake_group>
            <retractable>0</retractable>
        </contact>
        <contact type="BOGEY" name="Left Main">
            <location unit="IN">
                <x>58.2</x><y>-43.5</y><z>-18.46</z>
            </location>
            <static_friction>0.8</static_friction>
            <dynamic_friction>0.5</dynamic_friction>
            <rolling_friction>0.02</rolling_friction>
            <spring_coeff unit="LBS/FT">5400</spring_coeff>
            <damping_coeff unit="LBS/FT/SEC">160</damping_coeff>
            <max_steer unit="DEG">0</max_steer>
            <brake_group>LEFT</brake_group>
            <retractable>0</retractable>
        </contact>
        <contact type="BOGEY" name="Right Main">
            <location unit="IN">
                <x>58.2</x><y>43.5</y><z>-18.46</z>
            </location>
            <static_friction>0.8</static_friction>
            <dynamic_friction>0.5</dynamic_friction>
            <rolling_friction>0.02</rolling_friction>
            <spring_coeff unit="LBS/FT">5400</spring_coeff>
            <damping_coeff unit="LBS/FT/SEC">160</damping_coeff>
            <max_steer unit="DEG">0</max_steer>
            <brake_group>RIGHT</brake_group>
            <retractable>0</retractable>
        </contact>
    </ground_reactions>

    <propulsion>
        <engine file="electric147kW">
            <location unit="IN">
                <x>-20</x><y>0</y><z>36</z>
            </location>
            <orient unit="DEG">
                <pitch>0</pitch><roll>0</roll><yaw>0</yaw>
            </orient>
            <thruster file="prop_81in2v">
                <sense>1</sense>
                <location unit="IN">
                    <x>-40</x><y>0</y><z>36</z>
                </location>
                <orient unit="DEG">
                    <pitch>0</pitch><roll>0</roll><yaw>0</yaw>
                </orient>
            </thruster>
        </engine>
    </propulsion>

    <flight_control name="FCS">
        <channel name="Throttle">
            <fcs_function name="fcs/throttle-pos-norm">
                <function>
                    <property>fcs/throttle-cmd-norm</property>
                </function>
                <output>fcs/throttle-pos-norm</output>
            </fcs_function>
        </channel>
    </flight_control>

    <aerodynamics>
        <axis name="LIFT">
            <function name="aero/coefficient/CLalpha">
                <description>Lift_due_to_alpha</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <property>metrics/Sw-sqft</property>
                    <property>aero/alpha-rad</property>
                    <value>5.0</value>
                </product>
            </function>
        </axis>
        <axis name="DRAG">
            <function name="aero/coefficient/CD0">
                <description>Drag_at_zero_lift</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <property>metrics/Sw-sqft</property>
                    <value>0.025</value>
                </product>
            </function>
        </axis>
    </aerodynamics>
</fdm_config>
"""
        # Create aircraft subdirectory in sandbox
        aircraft_dir = os.path.join(self.sandbox(), "test_electric")
        os.makedirs(aircraft_dir, exist_ok=True)

        # Write aircraft file
        aircraft_file = os.path.join(aircraft_dir, "test_electric.xml")
        with open(aircraft_file, "w") as f:
            f.write(aircraft_xml)

        return "test_electric"

    @pytest.mark.propulsion
    def test_electric_motor_loading(self):
        """Test that electric motor loads correctly."""
        aircraft_name = self.create_electric_aircraft()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        # Verify engine loaded
        self.assertTrue(
            fdm.get_property_manager().hasNode("propulsion/engine[0]"),
            "Electric motor should be loaded as propulsion/engine[0]",
        )

        del fdm

    @pytest.mark.propulsion
    def test_electric_motor_power_output(self):
        """Test electric motor power output at full throttle."""
        aircraft_name = self.create_electric_aircraft()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        # Electric motors are simpler - just set throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0

        # Run to stabilize
        for _ in range(100):
            fdm.run()

        # Check power output
        power_hp = fdm["propulsion/engine/power-hp"]
        # 147kW = ~197 HP
        self.assertGreater(power_hp, 100, "Motor should produce significant power")

        del fdm

    @pytest.mark.propulsion
    def test_electric_zero_throttle(self):
        """Test that electric motor produces no power at zero throttle."""
        aircraft_name = self.create_electric_aircraft()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        # Set zero throttle
        fdm["fcs/throttle-cmd-norm"] = 0.0

        for _ in range(50):
            fdm.run()

        power_hp = fdm["propulsion/engine/power-hp"]
        # At zero throttle, power should be zero or very small
        self.assertLess(abs(power_hp), 5, "Motor should produce no power at zero throttle")

        del fdm

    @pytest.mark.propulsion
    def test_electric_thrust_generation(self):
        """Test that electric motor generates thrust through propeller."""
        aircraft_name = self.create_electric_aircraft()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        # Full throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0

        for _ in range(100):
            fdm.run()

        thrust = fdm["propulsion/engine/thrust-lbs"]
        self.assertGreater(thrust, 50, "Electric motor should generate thrust")

        del fdm

    @pytest.mark.propulsion
    def test_electric_propeller_rpm(self):
        """Test that propeller RPM responds to throttle on electric motor."""
        aircraft_name = self.create_electric_aircraft()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)
        fdm["ic/h-sl-ft"] = 3000
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        # Full throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0

        for _ in range(200):
            fdm.run()

        rpm = fdm["propulsion/engine/propeller-rpm"]
        self.assertGreater(rpm, 500, "Propeller should be spinning at significant RPM")

        del fdm


if __name__ == "__main__":
    RunTest(TestElectricBasic)
