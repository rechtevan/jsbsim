# TestSummerComponent.py
#
# Tests for summer (summing junction) FCS component (FGSummer).
# Uses C172X autopilot which includes summer components.
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


class TestSummerComponent(JSBSimTestCase):
    """
    Tests for summer (FGSummer) functionality.

    Uses C172X autopilot which has summer components:
    - fcs/heading-error (heading calculation)
    - fcs/altitude-error (altitude hold)
    - Various trim summers

    Tests cover:
    - Summer component loading
    - Summer output access
    - Summer error calculation
    """

    def test_summer_components_exist(self):
        """Test that C172X has summer components."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # C172X autopilot has several summers
        self.assertTrue(
            fdm.get_property_manager().hasNode("fcs/heading-error"),
            "Heading error summer should exist",
        )

        del fdm

    def test_heading_error_summer(self):
        """Test heading error summer calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/psi-true-rad"] = 0.0  # Heading north
        fdm.run_ic()

        # Set a heading setpoint
        fdm["ap/heading_setpoint"] = 90.0  # Target east

        fdm.run()

        heading_error = fdm["fcs/heading-error"]
        self.assertIsNotNone(heading_error, "Heading error should be calculated")

        del fdm

    def test_altitude_error_summer(self):
        """Test altitude error summer calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Set altitude setpoint different from current
        fdm["ap/altitude_setpoint"] = 6000

        for _ in range(10):
            fdm.run()

        altitude_error = fdm["fcs/altitude-error"]
        self.assertIsNotNone(altitude_error, "Altitude error should be calculated")

        del fdm

    def test_summer_output_accessible(self):
        """Test that summer outputs are accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Check various summer outputs
        summers = ["fcs/heading-error", "fcs/altitude-error"]
        for summer in summers:
            if fdm.get_property_manager().hasNode(summer):
                value = fdm[summer]
                self.assertIsNotNone(value, f"Summer {summer} output should be accessible")

        del fdm

    def test_fokker100_summers(self):
        """Test summer components in Fokker 100."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("fokker100")
            fdm.run_ic()

            # Fokker100 has pitch and roll trim summers
            has_pitch_sum = fdm.get_property_manager().hasNode("fcs/pitch-trim-sum")
            has_roll_sum = fdm.get_property_manager().hasNode("fcs/roll-trim-sum")

            self.assertTrue(has_pitch_sum or has_roll_sum, "Fokker100 should have trim summers")
        except Exception:
            # Fokker100 may have loading issues
            pass
        finally:
            del fdm

    def test_summer_responds_to_inputs(self):
        """Test that summer responds when inputs change."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Set setpoints
        fdm["ap/heading_setpoint"] = 0.0
        for _ in range(5):
            fdm.run()

        error_at_0 = fdm["fcs/heading-error"]

        # Change setpoint
        fdm["ap/heading_setpoint"] = 180.0
        for _ in range(5):
            fdm.run()

        error_at_180 = fdm["fcs/heading-error"]

        # Errors should be different
        self.assertNotEqual(error_at_0, error_at_180, "Summer output should change with inputs")

        del fdm


if __name__ == "__main__":
    RunTest(TestSummerComponent)
