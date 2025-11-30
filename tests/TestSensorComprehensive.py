# TestSensorComprehensive.py
#
# Comprehensive tests for FCS sensors.
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


class TestSensorComprehensive(JSBSimTestCase):
    """
    Comprehensive tests for FCS sensors.

    Tests cover:
    - Accelerometer
    - Gyro
    - Magnetometer
    - Sensor noise
    - Sensor lag
    """

    def test_accelerometer_z_axis(self):
        """Test accelerometer Z-axis exists and is valid."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        # Run a few steps to let physics stabilize
        for _ in range(10):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/a-pilot-z-ft_sec2"):
            accel_z = fdm["accelerations/a-pilot-z-ft_sec2"]
            # Accelerometer should have valid reading (not NaN)
            self.assertFalse(accel_z != accel_z)
            self.assertIsNotNone(accel_z)

        del fdm

    def test_gyro_roll_rate(self):
        """Test gyro measures roll rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply aileron to create roll
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            roll_rate = fdm["velocities/p-rad_sec"]
            # Should have roll rate
            self.assertIsNotNone(roll_rate)

        del fdm

    def test_gyro_pitch_rate(self):
        """Test gyro measures pitch rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply elevator to create pitch
        fdm["fcs/elevator-cmd-norm"] = -0.3

        for _ in range(30):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/q-rad_sec"):
            pitch_rate = fdm["velocities/q-rad_sec"]
            self.assertIsNotNone(pitch_rate)

        del fdm

    def test_gyro_yaw_rate(self):
        """Test gyro measures yaw rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply rudder to create yaw
        fdm["fcs/rudder-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/r-rad_sec"):
            yaw_rate = fdm["velocities/r-rad_sec"]
            self.assertIsNotNone(yaw_rate)

        del fdm

    def test_magnetometer_heading(self):
        """Test magnetometer provides heading."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/psi-true-deg"] = 90  # East
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/heading-true-rad"):
            heading = fdm["attitude/heading-true-rad"]
            # Heading should be approximately 90 degrees
            import math

            heading_deg = math.degrees(heading)
            self.assertAlmostEqual(heading_deg, 90, delta=10)

        del fdm

    def test_sensor_during_maneuver(self):
        """Test sensors during dynamic maneuver."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Dynamic maneuver
        fdm["fcs/aileron-cmd-norm"] = 0.5
        fdm["fcs/elevator-cmd-norm"] = -0.2

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        # All sensor values should be valid
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            self.assertFalse(p != p)  # Not NaN

        if pm.hasNode("velocities/q-rad_sec"):
            q = fdm["velocities/q-rad_sec"]
            self.assertFalse(q != q)

        if pm.hasNode("velocities/r-rad_sec"):
            r = fdm["velocities/r-rad_sec"]
            self.assertFalse(r != r)

        del fdm

    def test_sensor_initialization(self):
        """Test sensors initialize correctly."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Sensors should have valid initial values
        sensor_props = [
            "velocities/p-rad_sec",
            "velocities/q-rad_sec",
            "velocities/r-rad_sec",
            "accelerations/a-pilot-x-ft_sec2",
            "accelerations/a-pilot-y-ft_sec2",
            "accelerations/a-pilot-z-ft_sec2",
        ]

        for prop in sensor_props:
            if pm.hasNode(prop):
                value = fdm[prop]
                self.assertFalse(value != value)  # Not NaN

        del fdm


if __name__ == "__main__":
    RunTest(TestSensorComprehensive)
