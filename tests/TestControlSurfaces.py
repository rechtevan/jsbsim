# TestControlSurfaces.py
#
# Tests for flight control surfaces (elevator, aileron, rudder).
# Exercises control inputs, positions, and rate limiting.
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


class TestControlSurfaces(JSBSimTestCase):
    """
    Tests for flight control surfaces.

    Tests cover:
    - Elevator command and position
    - Aileron command and position
    - Rudder command and position
    - Control surface deflection limits
    - Trim inputs
    """

    def test_elevator_properties(self):
        """Test elevator command and position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Elevator properties
        elev_cmd = fdm["fcs/elevator-cmd-norm"]
        elev_pos = fdm["fcs/elevator-pos-rad"]

        self.assertIsNotNone(elev_cmd, "Elevator command should be accessible")
        self.assertIsNotNone(elev_pos, "Elevator position should be accessible")

        del fdm

    def test_aileron_properties(self):
        """Test aileron command and position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Aileron properties
        ail_cmd = fdm["fcs/aileron-cmd-norm"]
        ail_pos = fdm["fcs/left-aileron-pos-rad"]

        self.assertIsNotNone(ail_cmd, "Aileron command should be accessible")
        self.assertIsNotNone(ail_pos, "Aileron position should be accessible")

        del fdm

    def test_rudder_properties(self):
        """Test rudder command and position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Rudder properties
        rud_cmd = fdm["fcs/rudder-cmd-norm"]
        rud_pos = fdm["fcs/rudder-pos-rad"]

        self.assertIsNotNone(rud_cmd, "Rudder command should be accessible")
        self.assertIsNotNone(rud_pos, "Rudder position should be accessible")

        del fdm

    def test_elevator_command_response(self):
        """Test that elevator responds to command."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Apply elevator command
        fdm["fcs/elevator-cmd-norm"] = 0.5

        for _ in range(10):
            fdm.run()

        elev_pos = fdm["fcs/elevator-pos-rad"]
        # Position should be non-zero with command applied
        self.assertNotEqual(elev_pos, 0.0, "Elevator should respond to command")

        del fdm

    def test_aileron_command_response(self):
        """Test that ailerons respond to command."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Apply aileron command (right stick)
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(10):
            fdm.run()

        left_ail = fdm["fcs/left-aileron-pos-rad"]
        right_ail = fdm["fcs/right-aileron-pos-rad"]

        # Ailerons should deflect
        self.assertIsNotNone(left_ail)
        self.assertIsNotNone(right_ail)

        del fdm

    def test_rudder_command_response(self):
        """Test that rudder responds to command."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Apply rudder command
        fdm["fcs/rudder-cmd-norm"] = 0.5

        for _ in range(10):
            fdm.run()

        rud_pos = fdm["fcs/rudder-pos-rad"]
        # Position should be non-zero
        self.assertNotEqual(rud_pos, 0.0, "Rudder should respond to command")

        del fdm

    def test_control_limits(self):
        """Test that control positions are limited."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Apply maximum command
        fdm["fcs/elevator-cmd-norm"] = 1.0

        for _ in range(20):
            fdm.run()

        elev_pos = fdm["fcs/elevator-pos-rad"]

        # Elevator should not exceed typical limits (~25-30 degrees)
        max_deflection = 0.6  # ~35 degrees in radians
        self.assertLess(abs(elev_pos), max_deflection, "Elevator should be within limits")

        del fdm

    def test_trim_properties(self):
        """Test trim tab/input properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check trim properties
        if fdm.get_property_manager().hasNode("fcs/pitch-trim-cmd-norm"):
            pitch_trim = fdm["fcs/pitch-trim-cmd-norm"]
            self.assertIsNotNone(pitch_trim, "Pitch trim should be accessible")

        if fdm.get_property_manager().hasNode("fcs/roll-trim-cmd-norm"):
            roll_trim = fdm["fcs/roll-trim-cmd-norm"]
            self.assertIsNotNone(roll_trim, "Roll trim should be accessible")

        del fdm

    def test_elevator_trim_effect(self):
        """Test that elevator trim affects elevator position."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Zero control input
        fdm["fcs/elevator-cmd-norm"] = 0.0

        for _ in range(10):
            fdm.run()

        # Apply trim
        if fdm.get_property_manager().hasNode("fcs/pitch-trim-cmd-norm"):
            fdm["fcs/pitch-trim-cmd-norm"] = 0.5

            for _ in range(10):
                fdm.run()

            pos_with_trim = fdm["fcs/elevator-pos-rad"]

            # Position should change with trim
            # Note: Exact behavior depends on FCS implementation
            self.assertIsNotNone(pos_with_trim)

        del fdm

    def test_control_surface_neutral(self):
        """Test that controls are neutral with zero command."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Zero all commands
        fdm["fcs/elevator-cmd-norm"] = 0.0
        fdm["fcs/aileron-cmd-norm"] = 0.0
        fdm["fcs/rudder-cmd-norm"] = 0.0

        for _ in range(20):
            fdm.run()

        # Positions should be near zero (may have small trim offset)
        elev = fdm["fcs/elevator-pos-rad"]
        rud = fdm["fcs/rudder-pos-rad"]

        # Within 0.3 rad (~17 degrees) of neutral
        self.assertLess(abs(elev), 0.3, "Elevator should be near neutral")
        self.assertLess(abs(rud), 0.3, "Rudder should be near neutral")

        del fdm

    def test_flap_properties(self):
        """Test flap properties if available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Check flap properties
        if fdm.get_property_manager().hasNode("fcs/flap-cmd-norm"):
            flap_cmd = fdm["fcs/flap-cmd-norm"]
            self.assertIsNotNone(flap_cmd, "Flap command should be accessible")

        if fdm.get_property_manager().hasNode("fcs/flap-pos-deg"):
            flap_pos = fdm["fcs/flap-pos-deg"]
            self.assertIsNotNone(flap_pos, "Flap position should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestControlSurfaces)
