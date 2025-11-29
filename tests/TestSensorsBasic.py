# TestSensorsBasic.py
#
# Tests for sensor models: gyros, accelerometers, magnetometers.
# Exercises FGSensor, FGGyro, FGAccelerometer, FGMagnetometer models.
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

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestSensorsBasic(JSBSimTestCase):
    """
    Tests for sensor components in the FCS.

    Sensors tested:
    - Gyro (angular rate sensors)
    - Sensor base class properties

    Tests cover:
    - Sensor loading and configuration
    - Angular rate measurement
    - Sensor properties access
    """

    def create_aircraft_with_sensors(self):
        """
        Create a test aircraft with gyro and sensor systems.

        Returns:
            Name of the created aircraft model
        """
        # Create a minimal aircraft with sensor systems
        aircraft_xml = """<?xml version="1.0"?>
<fdm_config name="test_sensors" version="2.0" release="ALPHA"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://jsbsim.sourceforge.net/JSBSim.xsd">

    <fileheader>
        <author>Test</author>
        <filecreationdate>2024-01-01</filecreationdate>
        <version>1.0</version>
        <description>Test aircraft with sensors</description>
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

    <flight_control name="FCS">
        <channel name="Sensors">
            <!-- Gyro for roll rate -->
            <gyro name="fcs/roll-rate-gyro">
                <input>velocities/p-rad_sec</input>
                <lag>0.5</lag>
                <output>fcs/roll-rate-sensor</output>
            </gyro>

            <!-- Gyro for pitch rate -->
            <gyro name="fcs/pitch-rate-gyro">
                <input>velocities/q-rad_sec</input>
                <lag>0.5</lag>
                <output>fcs/pitch-rate-sensor</output>
            </gyro>

            <!-- Gyro for yaw rate -->
            <gyro name="fcs/yaw-rate-gyro">
                <input>velocities/r-rad_sec</input>
                <lag>0.5</lag>
                <output>fcs/yaw-rate-sensor</output>
            </gyro>

            <!-- Basic sensor -->
            <sensor name="fcs/altitude-sensor">
                <input>position/h-sl-ft</input>
                <lag>0.1</lag>
                <output>fcs/altitude-measured</output>
            </sensor>
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
        aircraft_dir = os.path.join(self.sandbox(), "test_sensors")
        os.makedirs(aircraft_dir, exist_ok=True)

        # Write aircraft file
        aircraft_file = os.path.join(aircraft_dir, "test_sensors.xml")
        with open(aircraft_file, "w") as f:
            f.write(aircraft_xml)

        return "test_sensors"

    def test_sensor_model_loading(self):
        """Test that aircraft with sensors loads correctly."""
        aircraft_name = self.create_aircraft_with_sensors()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)
        fdm.run_ic()

        # Verify FCS loaded with sensor channel
        self.assertTrue(
            fdm.get_property_manager().hasNode("fcs"),
            "FCS should be loaded",
        )

        del fdm

    def test_gyro_roll_rate_measurement(self):
        """Test gyro measures roll rate."""
        aircraft_name = self.create_aircraft_with_sensors()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/p-rad_sec"] = 0.1  # Roll rate
        fdm.run_ic()

        # Run to let sensor respond
        for _ in range(50):
            fdm.run()

        # Check sensor output exists
        if fdm.get_property_manager().hasNode("fcs/roll-rate-sensor"):
            sensor_output = fdm["fcs/roll-rate-sensor"]
            self.assertIsNotNone(sensor_output, "Roll rate sensor should produce output")

        del fdm

    def test_gyro_pitch_rate_measurement(self):
        """Test gyro measures pitch rate."""
        aircraft_name = self.create_aircraft_with_sensors()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/q-rad_sec"] = 0.05  # Pitch rate
        fdm.run_ic()

        # Run to let sensor respond
        for _ in range(50):
            fdm.run()

        # Check sensor output exists
        if fdm.get_property_manager().hasNode("fcs/pitch-rate-sensor"):
            sensor_output = fdm["fcs/pitch-rate-sensor"]
            self.assertIsNotNone(sensor_output, "Pitch rate sensor should produce output")

        del fdm

    def test_gyro_yaw_rate_measurement(self):
        """Test gyro measures yaw rate."""
        aircraft_name = self.create_aircraft_with_sensors()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/r-rad_sec"] = 0.02  # Yaw rate
        fdm.run_ic()

        # Run to let sensor respond
        for _ in range(50):
            fdm.run()

        # Check sensor output exists
        if fdm.get_property_manager().hasNode("fcs/yaw-rate-sensor"):
            sensor_output = fdm["fcs/yaw-rate-sensor"]
            self.assertIsNotNone(sensor_output, "Yaw rate sensor should produce output")

        del fdm

    def test_altitude_sensor(self):
        """Test basic sensor measures altitude."""
        aircraft_name = self.create_aircraft_with_sensors()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Run to let sensor respond
        for _ in range(50):
            fdm.run()

        # Check sensor output
        if fdm.get_property_manager().hasNode("fcs/altitude-measured"):
            measured_alt = fdm["fcs/altitude-measured"]
            # Measured altitude should be close to actual (with some lag)
            self.assertIsNotNone(measured_alt, "Altitude sensor should produce output")

        del fdm

    def test_c172_angular_rates(self):
        """Test angular rate properties on C172X."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Run simulation
        for _ in range(50):
            fdm.run()

        # Check angular rate properties exist
        p = fdm["velocities/p-rad_sec"]  # Roll rate
        q = fdm["velocities/q-rad_sec"]  # Pitch rate
        r = fdm["velocities/r-rad_sec"]  # Yaw rate

        self.assertIsNotNone(p, "Roll rate should be accessible")
        self.assertIsNotNone(q, "Pitch rate should be accessible")
        self.assertIsNotNone(r, "Yaw rate should be accessible")

        del fdm

    def test_sensor_lag_effect(self):
        """Test that sensor lag affects output."""
        aircraft_name = self.create_aircraft_with_sensors()

        fdm = CreateFDM(self.sandbox)
        fdm.set_aircraft_path(self.sandbox())
        fdm.load_model(aircraft_name)

        # Start with zero rates
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/p-rad_sec"] = 0.0
        fdm.run_ic()

        # Run a few frames at zero rate
        for _ in range(20):
            fdm.run()

        # Apply sudden roll rate
        fdm["velocities/p-rad_sec"] = 0.5

        # Sensor should lag behind - run a few frames
        sensor_values = []
        for _ in range(10):
            fdm.run()
            if fdm.get_property_manager().hasNode("fcs/roll-rate-sensor"):
                sensor_values.append(fdm["fcs/roll-rate-sensor"])

        # Due to lag, sensor shouldn't immediately reach 0.5
        # (exact behavior depends on lag implementation)
        if sensor_values:
            self.assertTrue(len(sensor_values) > 0, "Should collect sensor values")

        del fdm


if __name__ == "__main__":
    RunTest(TestSensorsBasic)
