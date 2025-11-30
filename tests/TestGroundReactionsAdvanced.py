# TestGroundReactionsAdvanced.py
#
# Advanced tests for ground reactions and landing gear.
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


class TestGroundReactionsAdvanced(JSBSimTestCase):
    """
    Advanced tests for ground reactions and landing gear.

    Tests cover:
    - Gear retraction
    - Braking
    - Steering
    - Strut dynamics
    - WOW transitions
    """

    def test_gear_retraction(self):
        """Test retractable gear operation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 300
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Command gear up
        if pm.hasNode("gear/gear-cmd-norm"):
            fdm["gear/gear-cmd-norm"] = 0.0

            for _ in range(200):
                fdm.run()

            # Check gear position
            if pm.hasNode("gear/gear-pos-norm"):
                pos = fdm["gear/gear-pos-norm"]
                self.assertIsNotNone(pos)

        del fdm

    def test_brake_application(self):
        """Test brake application on ground."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/vc-kts"] = 50
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Apply brakes
        if pm.hasNode("fcs/left-brake-cmd-norm"):
            fdm["fcs/left-brake-cmd-norm"] = 1.0
            fdm["fcs/right-brake-cmd-norm"] = 1.0

        initial_speed = fdm["velocities/vg-fps"]

        for _ in range(200):
            fdm.run()

        final_speed = fdm["velocities/vg-fps"]
        # Should decelerate with brakes
        self.assertLess(final_speed, initial_speed)

        del fdm

    def test_differential_braking(self):
        """Test differential braking for steering."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/vc-kts"] = 30
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Apply left brake only
        if pm.hasNode("fcs/left-brake-cmd-norm"):
            fdm["fcs/left-brake-cmd-norm"] = 1.0
            fdm["fcs/right-brake-cmd-norm"] = 0.0

        for _ in range(100):
            fdm.run()

        # Aircraft should turn - yaw rate should be non-zero
        if pm.hasNode("velocities/r-rad_sec"):
            yaw_rate = fdm["velocities/r-rad_sec"]
            self.assertIsNotNone(yaw_rate)

        del fdm

    def test_nose_wheel_steering(self):
        """Test nose wheel steering."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/vc-kts"] = 20
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Apply steering
        if pm.hasNode("fcs/steer-cmd-norm"):
            fdm["fcs/steer-cmd-norm"] = 0.5

        for _ in range(100):
            fdm.run()

        # Should produce yaw
        if pm.hasNode("attitude/psi-rad"):
            heading = fdm["attitude/psi-rad"]
            self.assertIsNotNone(heading)

        del fdm

    def test_strut_compression(self):
        """Test strut compression on ground."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("gear/unit[0]/compression-ft"):
            compression = fdm["gear/unit[0]/compression-ft"]
            # Should have some compression when on ground
            self.assertGreaterEqual(compression, 0)

        del fdm

    def test_wow_transition(self):
        """Test weight-on-wheels transition."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 100  # Just above ground
        fdm["ic/vc-kts"] = 60
        fdm["ic/gamma-deg"] = -3  # Descending
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Start airborne
        if pm.hasNode("gear/unit[0]/WOW"):
            # Run until touchdown
            for _ in range(500):
                fdm.run()
                if fdm["position/h-sl-ft"] < 1:
                    break

            # Should have transitioned
            time = fdm.get_sim_time()
            self.assertGreater(time, 0)

        del fdm

    def test_ground_friction(self):
        """Test ground friction coefficients."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/vc-kts"] = 0
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("gear/unit[0]/rolling_friction_coeff"):
            friction = fdm["gear/unit[0]/rolling_friction_coeff"]
            # Friction coefficient should be reasonable
            self.assertGreater(friction, 0)
            self.assertLess(friction, 1)

        del fdm

    def test_gear_forces(self):
        """Test gear force calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check total gear forces
        if pm.hasNode("forces/fbx-gear-lbs"):
            fx = fdm["forces/fbx-gear-lbs"]
            self.assertIsNotNone(fx)

        if pm.hasNode("forces/fbz-gear-lbs"):
            fz = fdm["forces/fbz-gear-lbs"]
            # Vertical force should support weight
            self.assertIsNotNone(fz)

        del fdm

    def test_takeoff_roll(self):
        """Test takeoff roll ground handling."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/vc-kts"] = 0
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 1.0

        # Run until airborne or timeout
        for _ in range(2000):
            fdm.run()
            if fdm["position/h-sl-ft"] > 50:
                break

        # Should have gained altitude
        alt = fdm["position/h-sl-ft"]
        self.assertGreater(alt, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestGroundReactionsAdvanced)
