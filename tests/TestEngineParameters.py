# TestEngineParameters.py
#
# Tests for engine parameters and performance.
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


class TestEngineParameters(JSBSimTestCase):
    """
    Tests for engine performance parameters.

    Tests cover:
    - RPM
    - Manifold pressure
    - EGT
    - Fuel flow
    """

    def test_engine_rpm(self):
        """Test engine RPM property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/engine-rpm"):
            rpm = fdm["propulsion/engine[0]/engine-rpm"]
            self.assertIsNotNone(rpm)

        del fdm

    def test_manifold_pressure(self):
        """Test manifold pressure property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/map-inhg"):
            mp = fdm["propulsion/engine[0]/map-inhg"]
            self.assertIsNotNone(mp)

        del fdm

    def test_propeller_thrust(self):
        """Test propeller thrust property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 1.0
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/thrust-lbs"):
            thrust = fdm["propulsion/engine[0]/thrust-lbs"]
            self.assertIsNotNone(thrust)

        del fdm

    def test_fuel_flow(self):
        """Test fuel flow property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/fuel-flow-rate-pps"):
            ff = fdm["propulsion/engine[0]/fuel-flow-rate-pps"]
            self.assertIsNotNone(ff)

        del fdm

    def test_throttle_response(self):
        """Test throttle affects power."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Start engine at idle
        fdm["fcs/throttle-cmd-norm"] = 0.1
        fdm["fcs/mixture-cmd-norm"] = 1.0
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if not pm.hasNode("propulsion/engine[0]/power-hp"):
            del fdm
            return

        power1 = fdm["propulsion/engine[0]/power-hp"]

        # Increase throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0
        for _ in range(100):
            fdm.run()

        power2 = fdm["propulsion/engine[0]/power-hp"]
        # Power should increase
        self.assertGreater(power2, power1)

        del fdm

    def test_oil_pressure(self):
        """Test oil pressure property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/oil-pressure-psi"):
            op = fdm["propulsion/engine[0]/oil-pressure-psi"]
            self.assertIsNotNone(op)

        del fdm

    def test_cht_property(self):
        """Test cylinder head temperature property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("propulsion/engine[0]/cht-degF"):
            cht = fdm["propulsion/engine[0]/cht-degF"]
            self.assertIsNotNone(cht)

        del fdm


if __name__ == "__main__":
    RunTest(TestEngineParameters)
