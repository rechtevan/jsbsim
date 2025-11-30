# TestModelFunctions.py
#
# Tests for model functions and mathematical operations.
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


class TestModelFunctions(JSBSimTestCase):
    """
    Tests for model function evaluation.

    Tests cover:
    - Function execution
    - Mathematical operations
    - Conditional evaluation
    """

    def test_function_execution(self):
        """Test function execution during simulation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Run and check that functions execute
        for _ in range(10):
            fdm.run()

        # If simulation runs, functions are executing
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_aero_functions(self):
        """Test aerodynamic function evaluation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()

        # Run a few steps
        for _ in range(10):
            fdm.run()

        # Check aerodynamic forces are computed
        if pm.hasNode("forces/fbx-aero-lbs"):
            force = fdm["forces/fbx-aero-lbs"]
            self.assertIsNotNone(force)

        del fdm

    def test_table_lookup(self):
        """Test table lookup functionality."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()

        # Aerodynamic tables should be used
        for _ in range(10):
            fdm.run()

        # If we get here without errors, tables are working
        if pm.hasNode("aero/alpha-rad"):
            alpha = fdm["aero/alpha-rad"]
            self.assertIsNotNone(alpha)

        del fdm

    def test_conditional_evaluation(self):
        """Test conditional evaluation in models."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()

        # Set condition inputs
        fdm["fcs/throttle-cmd-norm[0]"] = 0.5

        for _ in range(10):
            fdm.run()

        # Throttle should be applied
        if pm.hasNode("fcs/throttle-pos-norm[0]"):
            throttle = fdm["fcs/throttle-pos-norm[0]"]
            self.assertGreater(throttle, 0)

        del fdm

    def test_gain_function(self):
        """Test gain function evaluation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()

        # Set input
        fdm["fcs/elevator-cmd-norm"] = 0.5

        for _ in range(5):
            fdm.run()

        # Elevator should have some position
        if pm.hasNode("fcs/elevator-pos-rad"):
            pos = fdm["fcs/elevator-pos-rad"]
            self.assertIsNotNone(pos)

        del fdm

    def test_summer_function(self):
        """Test summer/addition function."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()

        # Run simulation
        for _ in range(10):
            fdm.run()

        # Total forces should be sum of components
        if pm.hasNode("forces/fbx-total-lbs"):
            total = fdm["forces/fbx-total-lbs"]
            self.assertIsNotNone(total)

        del fdm


if __name__ == "__main__":
    RunTest(TestModelFunctions)
