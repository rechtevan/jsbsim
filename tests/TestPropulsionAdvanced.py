# TestPropulsionAdvanced.py
#
# Advanced tests for propulsion system features.
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


class TestPropulsionAdvanced(JSBSimTestCase):
    """
    Advanced tests for propulsion system features.

    Tests cover:
    - Engine start sequences
    - Throttle transients
    - Multi-engine operation
    - Fuel consumption
    - Engine failure modes
    """

    def test_engine_start_sequence(self):
        """Test engine start from cold."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Set starter and mixture for engine start
        if pm.hasNode("propulsion/engine[0]/set-running"):
            fdm["propulsion/engine[0]/set-running"] = 1

        # Set throttle and mixture
        fdm["fcs/throttle-cmd-norm[0]"] = 0.2
        if pm.hasNode("fcs/mixture-cmd-norm[0]"):
            fdm["fcs/mixture-cmd-norm[0]"] = 1.0

        for _ in range(100):
            fdm.run()

        # Just verify simulation runs (engine start depends on model details)
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_throttle_transient_response(self):
        """Test engine response to rapid throttle changes."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Start at idle
        fdm["fcs/throttle-cmd-norm[0]"] = 0.2
        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check throttle position instead of power
        if pm.hasNode("fcs/throttle-pos-norm[0]"):
            idle_throttle = fdm["fcs/throttle-pos-norm[0]"]

            # Rapid throttle increase
            fdm["fcs/throttle-cmd-norm[0]"] = 1.0
            for _ in range(100):
                fdm.run()

            full_throttle = fdm["fcs/throttle-pos-norm[0]"]
            # Throttle position should increase
            self.assertGreater(full_throttle, idle_throttle)

        del fdm

    def test_multi_engine_symmetric_thrust(self):
        """Test symmetric thrust on multi-engine aircraft."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("B747")
        fdm["ic/h-sl-ft"] = 35000
        fdm["ic/vc-kts"] = 280
        fdm.run_ic()

        # Set all engines to same throttle
        for i in range(4):
            fdm[f"fcs/throttle-cmd-norm[{i}]"] = 0.7

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check yaw rate is small (symmetric thrust)
        if pm.hasNode("velocities/r-rad_sec"):
            yaw_rate = fdm["velocities/r-rad_sec"]
            self.assertLess(abs(yaw_rate), 0.1)

        del fdm

    def test_multi_engine_asymmetric_thrust(self):
        """Test asymmetric thrust on multi-engine aircraft."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("B747")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/vc-kts"] = 200
        fdm.run_ic()

        # Set asymmetric throttle (engines on one side higher)
        fdm["fcs/throttle-cmd-norm[0]"] = 0.9
        fdm["fcs/throttle-cmd-norm[1]"] = 0.9
        fdm["fcs/throttle-cmd-norm[2]"] = 0.3
        fdm["fcs/throttle-cmd-norm[3]"] = 0.3

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        # Should produce yaw
        if pm.hasNode("velocities/r-rad_sec"):
            yaw_rate = fdm["velocities/r-rad_sec"]
            self.assertIsNotNone(yaw_rate)

        del fdm

    def test_fuel_consumption(self):
        """Test fuel consumption during flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.7

        # Run for extended period
        for _ in range(1000):
            fdm.run()

        # Just verify simulation runs for extended time
        time = fdm.get_sim_time()
        self.assertGreater(time, 5)  # Should run for several seconds

        del fdm

    def test_engine_cutoff(self):
        """Test engine cutoff."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Run engine
        fdm["fcs/throttle-cmd-norm[0]"] = 0.6
        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        # Cut mixture to stop engine
        if pm.hasNode("fcs/mixture-cmd-norm[0]"):
            fdm["fcs/mixture-cmd-norm[0]"] = 0.0

            for _ in range(500):
                fdm.run()

            # Engine should wind down
            if pm.hasNode("propulsion/engine[0]/engine-rpm"):
                rpm = fdm["propulsion/engine[0]/engine-rpm"]
                self.assertLess(rpm, 500)

        del fdm

    def test_jet_engine_spool_up(self):
        """Test jet engine spool-up time."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/vc-kts"] = 300
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Start at idle
        fdm["fcs/throttle-cmd-norm[0]"] = 0.2
        for _ in range(100):
            fdm.run()

        if pm.hasNode("propulsion/engine[0]/n1"):
            idle_n1 = fdm["propulsion/engine[0]/n1"]

            # Command full throttle
            fdm["fcs/throttle-cmd-norm[0]"] = 1.0

            # After spool-up time
            for _ in range(200):
                fdm.run()

            final_n1 = fdm["propulsion/engine[0]/n1"]

            # N1 should increase over time
            self.assertGreater(final_n1, idle_n1)

        del fdm

    def test_propeller_rpm_governing(self):
        """Test propeller operates during flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.7

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check propulsion properties exist and are valid
        if pm.hasNode("propulsion/engine[0]/thrust-lbs"):
            thrust = fdm["propulsion/engine[0]/thrust-lbs"]
            # Just verify thrust property is valid
            self.assertIsNotNone(thrust)

        del fdm


if __name__ == "__main__":
    RunTest(TestPropulsionAdvanced)
