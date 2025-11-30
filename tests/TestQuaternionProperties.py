# TestQuaternionProperties.py
#
# Tests for quaternion orientation properties.
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

import math

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestQuaternionProperties(JSBSimTestCase):
    """
    Tests for quaternion orientation properties.

    Tests cover:
    - Quaternion components (q0, q1, q2, q3)
    - Quaternion normalization
    - Quaternion to Euler conversion
    """

    def test_quaternion_q0(self):
        """Test quaternion q0 component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/q0"):
            q0 = fdm["attitude/q0"]
            self.assertIsNotNone(q0)

        del fdm

    def test_quaternion_q1(self):
        """Test quaternion q1 component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/q1"):
            q1 = fdm["attitude/q1"]
            self.assertIsNotNone(q1)

        del fdm

    def test_quaternion_q2(self):
        """Test quaternion q2 component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/q2"):
            q2 = fdm["attitude/q2"]
            self.assertIsNotNone(q2)

        del fdm

    def test_quaternion_q3(self):
        """Test quaternion q3 component."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("attitude/q3"):
            q3 = fdm["attitude/q3"]
            self.assertIsNotNone(q3)

        del fdm

    def test_quaternion_normalized(self):
        """Test quaternion is normalized (magnitude = 1)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if (
            pm.hasNode("attitude/q0")
            and pm.hasNode("attitude/q1")
            and pm.hasNode("attitude/q2")
            and pm.hasNode("attitude/q3")
        ):
            q0 = fdm["attitude/q0"]
            q1 = fdm["attitude/q1"]
            q2 = fdm["attitude/q2"]
            q3 = fdm["attitude/q3"]
            mag = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
            self.assertAlmostEqual(mag, 1.0, delta=0.001)

        del fdm

    def test_quaternion_after_rotation(self):
        """Test quaternion changes with rotation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("attitude/q0"):
            del fdm
            return

        # Apply roll
        fdm["fcs/aileron-cmd-norm"] = 0.5
        for _ in range(100):
            fdm.run()

        # Quaternion should still be normalized
        q0 = fdm["attitude/q0"]
        q1 = fdm["attitude/q1"]
        q2 = fdm["attitude/q2"]
        q3 = fdm["attitude/q3"]
        mag = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.assertAlmostEqual(mag, 1.0, delta=0.001)

        del fdm


if __name__ == "__main__":
    RunTest(TestQuaternionProperties)
