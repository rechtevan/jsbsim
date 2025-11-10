# fpectl.py
#
# Check that the module fpectl catches floating point exceptions
#
# Copyright (c) 2018 Bertrand Coconnier
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
import sys

sys.path.append(os.getcwd())

try:
    import fpectl
except ImportError:
    # fpectl is not available in Python 3.12+
    fpectl = None

import unittest  # noqa: E402

from JSBSim_utils import RunTest  # noqa: E402


class check_fpectl(unittest.TestCase):
    def testModule(self):
        if fpectl is None:
            self.skipTest("fpectl module not available (Python 3.12+)")
        # Check that FP exceptions are not caught by default
        fpectl.test_sigfpe()

        # Check that once fpectl is turned on, a Python exception is raised when
        # a floating point error occurs.
        with self.assertRaises(FloatingPointError):
            fpectl.turnon_sigfpe()
            fpectl.test_sigfpe()


RunTest(check_fpectl)
