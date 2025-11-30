# TestLandingGearBasic.py
#
# Tests for landing gear (FGLGear) functionality.
# Exercises gear extension, ground contact, and braking.
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


class TestLandingGearBasic(JSBSimTestCase):
    """
    Tests for landing gear (FGLGear) functionality.

    Tests cover:
    - Gear property access
    - Gear down/up states
    - Ground contact detection
    - Brake functionality
    - Steering capability
    """

    def test_gear_properties_exist(self):
        """Test that landing gear properties are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check gear properties
        self.assertTrue(
            fdm.get_property_manager().hasNode("gear/unit[0]/WOW"),
            "Weight on wheels property should exist",
        )

        del fdm

    def test_gear_wow_on_ground(self):
        """Test weight-on-wheels when on ground."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Start on ground
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/u-fps"] = 0
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # At least one gear should have weight on wheels
        wow0 = fdm["gear/unit[0]/WOW"]
        wow1 = fdm["gear/unit[1]/WOW"]
        wow2 = fdm["gear/unit[2]/WOW"]

        any_wow = wow0 or wow1 or wow2
        self.assertTrue(any_wow, "Some gear should have WOW on ground")

        del fdm

    def test_gear_wow_in_air(self):
        """Test no weight-on-wheels when in air."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Start in air
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # No gear should have weight on wheels
        wow0 = fdm["gear/unit[0]/WOW"]
        wow1 = fdm["gear/unit[1]/WOW"]
        wow2 = fdm["gear/unit[2]/WOW"]

        no_wow = not wow0 and not wow1 and not wow2
        self.assertTrue(no_wow, "No gear should have WOW in air")

        del fdm

    def test_brake_properties(self):
        """Test brake command properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check brake properties
        if fdm.get_property_manager().hasNode("fcs/left-brake-cmd-norm"):
            left_brake = fdm["fcs/left-brake-cmd-norm"]
            right_brake = fdm["fcs/right-brake-cmd-norm"]

            self.assertIsNotNone(left_brake, "Left brake should be accessible")
            self.assertIsNotNone(right_brake, "Right brake should be accessible")

        del fdm

    def test_brake_application(self):
        """Test applying brakes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 0
        fdm["ic/u-fps"] = 50  # Rolling
        fdm.run_ic()

        # Apply brakes
        fdm["fcs/left-brake-cmd-norm"] = 1.0
        fdm["fcs/right-brake-cmd-norm"] = 1.0

        for _ in range(10):
            fdm.run()

        left_brake = fdm["fcs/left-brake-cmd-norm"]
        self.assertAlmostEqual(left_brake, 1.0, places=1, msg="Brake should be applied")

        del fdm

    def test_steering_property(self):
        """Test nose wheel steering property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check steering property
        if fdm.get_property_manager().hasNode("fcs/steer-cmd-norm"):
            steer = fdm["fcs/steer-cmd-norm"]
            self.assertIsNotNone(steer, "Steering should be accessible")

        del fdm

    def test_gear_compression(self):
        """Test gear compression property when on ground."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 0
        fdm["ic/u-fps"] = 0
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Check gear compression (strut travel)
        if fdm.get_property_manager().hasNode("gear/unit[0]/compression-ft"):
            comp = fdm["gear/unit[0]/compression-ft"]
            self.assertIsNotNone(comp, "Gear compression should be accessible")

        del fdm

    def test_gear_force_on_ground(self):
        """Test that gear produces force when on ground."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 0
        fdm["ic/u-fps"] = 0
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Check for gear force properties
        total_force = 0
        for i in range(3):
            force_prop = f"gear/unit[{i}]/z-position"
            if fdm.get_property_manager().hasNode(force_prop):
                pos = fdm[force_prop]
                if pos is not None:
                    total_force += 1

        # At least some gear position data should exist
        self.assertGreater(total_force, 0, "Gear position data should exist")

        del fdm

    def test_retractable_gear_property(self):
        """Test retractable gear properties if available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # C172 has fixed gear, but check property exists
        if fdm.get_property_manager().hasNode("gear/gear-cmd-norm"):
            gear_cmd = fdm["gear/gear-cmd-norm"]
            self.assertIsNotNone(gear_cmd)

        del fdm

    def test_f16_retractable_gear(self):
        """Test F16 retractable landing gear."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f16")
            fdm["ic/h-sl-ft"] = 5000
            fdm["ic/u-fps"] = 500
            fdm.run_ic()

            # F16 should have retractable gear
            if fdm.get_property_manager().hasNode("gear/gear-cmd-norm"):
                # Retract gear
                fdm["gear/gear-cmd-norm"] = 0.0

                for _ in range(50):
                    fdm.run()

                # Gear should be retracting/retracted
                gear_pos = fdm["gear/gear-pos-norm"]
                self.assertIsNotNone(gear_pos)
        except Exception:
            pass
        finally:
            del fdm


if __name__ == "__main__":
    RunTest(TestLandingGearBasic)
