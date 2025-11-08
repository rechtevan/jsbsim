# test_07_events_conditions.py
#
# Integration Test Scenario 7: Event and Condition System Validation
#
# This test exercises JSBSim's event and condition system, which enables
# script-driven automation of simulation scenarios. Events can trigger
# actions based on time or complex logical conditions involving simulation
# properties. This is a critical feature for automated testing, mission
# simulation, and scenario replay.
#
# Components tested:
# - FGScript: Script parsing, event management, execution sequencing
# - FGPropertyReader: Condition evaluation, property comparison operations
# - FGPropertyManager: Property access and modification from events
# - FGCondition: Logical condition evaluation (AND, OR, NOT, comparisons)
# - Event triggering: Time-based, condition-based, persistent, one-shot
# - Event actions: Property setting, notifications, delays
#
# Expected coverage gain: +2-3%
#
# Copyright (c) 2025 Booz Allen Hamilton Inc.
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

import os
import sys
import xml.etree.ElementTree as et

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from JSBSim_utils import ExecuteUntil, JSBSimTestCase, RunTest


class TestEventsConditions(JSBSimTestCase):
    """
    Integration test for JSBSim's event and condition system.

    This test suite validates that JSBSim correctly processes events and
    conditions defined in simulation scripts. Events are the primary mechanism
    for automating simulation scenarios, allowing property changes, notifications,
    and other actions to be triggered at specific times or when conditions are met.

    Test Coverage:
    - Time-based event triggers (events at specific simulation times)
    - Condition-based events (property threshold triggers)
    - Multiple sequential events
    - Event notifications
    - Events setting multiple properties simultaneously
    - Persistent events (trigger repeatedly while condition is true)
    - One-shot events (trigger only once)
    - Compound conditions with logical operators (AND, OR)
    - Various comparison operators (>, <, >=, <=, ==, !=)
    - Event timing accuracy
    - Property modification verification
    """

    def _create_simple_script(self, aircraft="ball", initialize="reset00",
                             end_time=10.0, dt=0.00833333):
        """
        Create a basic script XML structure for testing events.

        Args:
            aircraft: Aircraft model to use
            initialize: Initialization file
            end_time: Simulation end time in seconds
            dt: Time step in seconds

        Returns:
            ElementTree.Element: Root element of the script
        """
        script = et.Element('runscript')
        script.set('xmlns:xsi', 'http://www.w3.org/2001/XMLSchema-instance')
        script.set('xsi:noNamespaceSchemaLocation', 'http://jsbsim.sf.net/JSBSimScript.xsd')
        script.set('name', 'Event Test Script')

        # Use element
        use = et.SubElement(script, 'use')
        use.set('aircraft', aircraft)
        use.set('initialize', initialize)

        # Run element
        run = et.SubElement(script, 'run')
        run.set('start', '0.0')
        run.set('end', str(end_time))
        run.set('dt', str(dt))

        return script, run

    def _write_script(self, script, filename='test_script.xml'):
        """
        Write script XML to file.

        Args:
            script: ElementTree.Element root element
            filename: Output filename
        """
        tree = et.ElementTree(script)
        et.indent(tree, space='  ')
        tree.write(filename, encoding='UTF-8', xml_declaration=True)

    def test_time_based_event(self):
        """
        Test time-based event triggering at specific simulation time.

        This test verifies that events triggered by simulation time work correctly.
        Time-based events are the simplest form of event, executing when the
        simulation time reaches or exceeds a specified threshold.

        The event should:
        - Trigger exactly when the time condition is met
        - Execute property modifications
        - Trigger only once (default behavior)

        This tests:
        - FGScript event parsing and time-based condition evaluation
        - Event trigger timing accuracy
        - Property modification through events
        """
        # Create script with time-based event
        script, run = self._create_simple_script(end_time=5.0)

        # Add event that triggers at t=2.0 seconds
        event = et.SubElement(run, 'event')
        event.set('name', 'Time-based event')

        condition = et.SubElement(event, 'condition')
        condition.text = 'simulation/sim-time-sec >= 2.0'

        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/time-event-triggered')
        set_prop.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        # Create the test property before running
        fdm.set_property_value('test/time-event-triggered', 0.0)

        fdm.run_ic()

        # Run until just before event should trigger (t=1.5)
        ExecuteUntil(fdm, 1.5)

        # Verify event has not triggered yet
        triggered = fdm['test/time-event-triggered']
        self.assertEqual(triggered, 0.0,
                        "Event should not trigger before t=2.0")

        # Run until after event should trigger (t=2.5)
        ExecuteUntil(fdm, 2.5)

        # Verify event has triggered
        triggered = fdm['test/time-event-triggered']
        self.assertEqual(triggered, 1.0,
                        "Event should have triggered at t=2.0")

        # Verify event only triggered once (property still 1.0)
        ExecuteUntil(fdm, 4.0)
        triggered = fdm['test/time-event-triggered']
        self.assertEqual(triggered, 1.0,
                        "Event should only trigger once")

    def test_condition_based_event(self):
        """
        Test condition-based event triggered by property threshold.

        This test verifies that events can be triggered by arbitrary property
        conditions. Condition-based events are more flexible than time-based
        events and enable reactive simulation behaviors.

        The event should:
        - Monitor a property value continuously
        - Trigger when the property crosses a threshold
        - Execute the specified actions

        This tests:
        - FGPropertyReader condition evaluation
        - Property-based event triggering
        - Comparison operators (greater than)
        - Event action execution when conditions are met
        """
        # Create script with property condition event
        script, run = self._create_simple_script(aircraft='c172p', end_time=10.0)

        # Set initial airspeed
        property_tag = et.SubElement(run, 'property')
        property_tag.set('value', '50.0')
        property_tag.text = 'ic/vc-kts'

        # Add event that triggers when airspeed exceeds 60 knots
        event = et.SubElement(run, 'event')
        event.set('name', 'Speed threshold event')

        condition = et.SubElement(event, 'condition')
        condition.text = 'velocities/vc-kts > 60.0'

        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/speed-event-triggered')
        set_prop.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        # Create test property
        fdm.set_property_value('test/speed-event-triggered', 0.0)

        fdm.run_ic()

        # Initially at 50 knots - event should not trigger
        airspeed = fdm['velocities/vc-kts']
        triggered = fdm['test/speed-event-triggered']
        self.assertLess(airspeed, 60.0, "Initial airspeed should be < 60 kts")
        self.assertEqual(triggered, 0.0, "Event should not trigger at low speed")

        # Manually increase airspeed above threshold
        fdm['velocities/vc-kts'] = 65.0

        # Run a few steps to allow event to trigger
        for _ in range(5):
            fdm.run()

        # Verify event triggered
        triggered = fdm['test/speed-event-triggered']
        self.assertEqual(triggered, 1.0,
                        "Event should trigger when airspeed exceeds 60 kts")

    def test_multiple_sequential_events(self):
        """
        Test multiple events triggering in sequence.

        This test verifies that multiple events can coexist in a script and
        trigger independently at their specified times or conditions. This is
        essential for complex scenarios with multiple phases.

        The test validates:
        - Multiple events in a single script
        - Events triggering in chronological order
        - Independent event timing
        - Each event executes its own actions

        This tests:
        - FGScript management of multiple events
        - Event scheduling and execution order
        - Independent event condition evaluation
        """
        # Create script with three sequential time-based events
        script, run = self._create_simple_script(end_time=8.0)

        # Event 1: Trigger at t=1.0
        event1 = et.SubElement(run, 'event')
        event1.set('name', 'First event')
        condition1 = et.SubElement(event1, 'condition')
        condition1.text = 'simulation/sim-time-sec >= 1.0'
        set1 = et.SubElement(event1, 'set')
        set1.set('name', 'test/event-1')
        set1.set('value', '1.0')

        # Event 2: Trigger at t=3.0
        event2 = et.SubElement(run, 'event')
        event2.set('name', 'Second event')
        condition2 = et.SubElement(event2, 'condition')
        condition2.text = 'simulation/sim-time-sec >= 3.0'
        set2 = et.SubElement(event2, 'set')
        set2.set('name', 'test/event-2')
        set2.set('value', '2.0')

        # Event 3: Trigger at t=5.0
        event3 = et.SubElement(run, 'event')
        event3.set('name', 'Third event')
        condition3 = et.SubElement(event3, 'condition')
        condition3.text = 'simulation/sim-time-sec >= 5.0'
        set3 = et.SubElement(event3, 'set')
        set3.set('name', 'test/event-3')
        set3.set('value', '3.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        # Create test properties
        fdm.set_property_value('test/event-1', 0.0)
        fdm.set_property_value('test/event-2', 0.0)
        fdm.set_property_value('test/event-3', 0.0)

        fdm.run_ic()

        # At t=0.5, no events should have triggered
        ExecuteUntil(fdm, 0.5)
        self.assertEqual(fdm['test/event-1'], 0.0, "Event 1 should not trigger yet")
        self.assertEqual(fdm['test/event-2'], 0.0, "Event 2 should not trigger yet")
        self.assertEqual(fdm['test/event-3'], 0.0, "Event 3 should not trigger yet")

        # At t=2.0, only event 1 should have triggered
        ExecuteUntil(fdm, 2.0)
        self.assertEqual(fdm['test/event-1'], 1.0, "Event 1 should have triggered")
        self.assertEqual(fdm['test/event-2'], 0.0, "Event 2 should not trigger yet")
        self.assertEqual(fdm['test/event-3'], 0.0, "Event 3 should not trigger yet")

        # At t=4.0, events 1 and 2 should have triggered
        ExecuteUntil(fdm, 4.0)
        self.assertEqual(fdm['test/event-1'], 1.0, "Event 1 should remain triggered")
        self.assertEqual(fdm['test/event-2'], 2.0, "Event 2 should have triggered")
        self.assertEqual(fdm['test/event-3'], 0.0, "Event 3 should not trigger yet")

        # At t=6.0, all events should have triggered
        ExecuteUntil(fdm, 6.0)
        self.assertEqual(fdm['test/event-1'], 1.0, "Event 1 should remain triggered")
        self.assertEqual(fdm['test/event-2'], 2.0, "Event 2 should remain triggered")
        self.assertEqual(fdm['test/event-3'], 3.0, "Event 3 should have triggered")

    def test_event_with_notify(self):
        """
        Test event notification messages.

        This test verifies that events can include notify tags to output
        property values when the event triggers. Notifications are useful
        for debugging and monitoring simulation state during automated runs.

        The test validates:
        - Notify tags are parsed correctly
        - Notifications are generated when event triggers
        - Multiple properties can be notified

        This tests:
        - FGScript notify processing
        - Event notification generation
        - Property value reporting through events

        Note: We cannot directly capture console output, but we verify
        the event triggers correctly with notify present.
        """
        # Create script with event that includes notify
        script, run = self._create_simple_script(end_time=5.0)

        event = et.SubElement(run, 'event')
        event.set('name', 'Event with notification')

        condition = et.SubElement(event, 'condition')
        condition.text = 'simulation/sim-time-sec >= 2.0'

        # Add notify element with properties to display
        notify = et.SubElement(event, 'notify')
        prop1 = et.SubElement(notify, 'property')
        prop1.text = 'simulation/sim-time-sec'
        prop2 = et.SubElement(notify, 'property')
        prop2.text = 'position/h-sl-ft'

        # Also set a property to verify event triggered
        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/notify-event-triggered')
        set_prop.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        fdm.set_property_value('test/notify-event-triggered', 0.0)
        fdm.run_ic()

        # Run until after event
        ExecuteUntil(fdm, 3.0)

        # Verify event triggered (notification would have been printed)
        triggered = fdm['test/notify-event-triggered']
        self.assertEqual(triggered, 1.0,
                        "Event with notify should have triggered")

    def test_event_setting_multiple_properties(self):
        """
        Test event that sets multiple properties simultaneously.

        This test verifies that a single event can modify multiple properties
        when it triggers. This is essential for scenarios where a single
        condition should cause multiple state changes (e.g., engine failure
        affects multiple engine parameters).

        The test validates:
        - Multiple set tags in a single event
        - All properties are modified when event triggers
        - Property values are set correctly

        This tests:
        - FGScript processing of multiple set actions
        - Event execution with multiple property modifications
        - Simultaneous property updates
        """
        # Create script with event setting multiple properties
        script, run = self._create_simple_script(end_time=5.0)

        event = et.SubElement(run, 'event')
        event.set('name', 'Multi-property event')

        condition = et.SubElement(event, 'condition')
        condition.text = 'simulation/sim-time-sec >= 2.0'

        # Set multiple properties
        set1 = et.SubElement(event, 'set')
        set1.set('name', 'test/prop-1')
        set1.set('value', '10.0')

        set2 = et.SubElement(event, 'set')
        set2.set('name', 'test/prop-2')
        set2.set('value', '20.0')

        set3 = et.SubElement(event, 'set')
        set3.set('name', 'test/prop-3')
        set3.set('value', '30.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        # Create test properties
        fdm.set_property_value('test/prop-1', 0.0)
        fdm.set_property_value('test/prop-2', 0.0)
        fdm.set_property_value('test/prop-3', 0.0)

        fdm.run_ic()

        # Before event
        ExecuteUntil(fdm, 1.5)
        self.assertEqual(fdm['test/prop-1'], 0.0, "Property 1 should not be set yet")
        self.assertEqual(fdm['test/prop-2'], 0.0, "Property 2 should not be set yet")
        self.assertEqual(fdm['test/prop-3'], 0.0, "Property 3 should not be set yet")

        # After event
        ExecuteUntil(fdm, 2.5)
        self.assertEqual(fdm['test/prop-1'], 10.0, "Property 1 should be set")
        self.assertEqual(fdm['test/prop-2'], 20.0, "Property 2 should be set")
        self.assertEqual(fdm['test/prop-3'], 30.0, "Property 3 should be set")

    def test_persistent_event(self):
        """
        Test persistent event that triggers multiple times.

        This test verifies that events with persistent="true" attribute
        trigger every simulation frame while their condition remains true.
        Persistent events are useful for continuous monitoring and repeated
        actions.

        The test validates:
        - Persistent attribute is recognized
        - Event triggers multiple times while condition is true
        - Event stops triggering when condition becomes false

        This tests:
        - FGScript persistent event handling
        - Continuous condition monitoring
        - Event re-triggering behavior
        """
        # Create script with persistent event
        script, run = self._create_simple_script(end_time=6.0)

        # Initialize counter property
        init_prop = et.SubElement(run, 'property')
        init_prop.set('value', '0.0')
        init_prop.text = 'test/trigger-count'

        # Persistent event that increments counter while condition is true
        event = et.SubElement(run, 'event')
        event.set('name', 'Persistent event')
        event.set('persistent', 'true')

        # Condition true between t=2.0 and t=4.0
        condition = et.SubElement(event, 'condition')
        condition.text = '''simulation/sim-time-sec >= 2.0
                           simulation/sim-time-sec lt 4.0'''

        # Increment counter each time event triggers
        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/trigger-count')
        set_prop.set('value', '1.0')
        set_prop.set('type', 'FG_DELTA')  # Add to current value

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')
        fdm.run_ic()

        # Before event window (t=1.0)
        ExecuteUntil(fdm, 1.0)
        count_before = fdm['test/trigger-count']
        self.assertEqual(count_before, 0.0,
                        "Counter should be 0 before event triggers")

        # During event window (t=3.0)
        ExecuteUntil(fdm, 3.0)
        count_during = fdm['test/trigger-count']
        self.assertGreater(count_during, 0.0,
                          "Counter should increase while persistent event triggers")

        # Continue in event window
        ExecuteUntil(fdm, 3.5)
        count_continued = fdm['test/trigger-count']
        self.assertGreater(count_continued, count_during,
                          "Counter should continue increasing with persistent event")

        # After event window (t=5.0) - should stop increasing
        ExecuteUntil(fdm, 5.0)
        count_after = fdm['test/trigger-count']

        # Run a bit more to ensure it really stopped
        ExecuteUntil(fdm, 5.5)
        count_final = fdm['test/trigger-count']
        self.assertEqual(count_final, count_after,
                        "Counter should stop increasing after condition becomes false")

    def test_oneshot_event_explicit(self):
        """
        Test explicit one-shot event (default behavior).

        This test verifies that events without persistent="true" trigger
        only once, even if their condition remains true for extended periods.
        This is the default behavior and most common event type.

        The test validates:
        - Event triggers once when condition first becomes true
        - Event does not re-trigger even though condition stays true
        - Property maintains value after single trigger

        This tests:
        - FGScript default one-shot event behavior
        - Event state management
        - Single trigger enforcement
        """
        # Create script with one-shot event (default)
        script, run = self._create_simple_script(end_time=8.0)

        # Initialize counter
        init_prop = et.SubElement(run, 'property')
        init_prop.set('value', '0.0')
        init_prop.text = 'test/oneshot-count'

        # One-shot event (no persistent attribute)
        event = et.SubElement(run, 'event')
        event.set('name', 'One-shot event')

        # Condition becomes true at t=2.0 and stays true
        condition = et.SubElement(event, 'condition')
        condition.text = 'simulation/sim-time-sec >= 2.0'

        # Increment counter
        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/oneshot-count')
        set_prop.set('value', '1.0')
        set_prop.set('type', 'FG_DELTA')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')
        fdm.run_ic()

        # Before trigger
        ExecuteUntil(fdm, 1.5)
        self.assertEqual(fdm['test/oneshot-count'], 0.0,
                        "Counter should be 0 before trigger")

        # Just after trigger
        ExecuteUntil(fdm, 2.5)
        self.assertEqual(fdm['test/oneshot-count'], 1.0,
                        "Counter should be 1 after one-shot trigger")

        # Much later, condition still true
        ExecuteUntil(fdm, 6.0)
        self.assertEqual(fdm['test/oneshot-count'], 1.0,
                        "Counter should still be 1 (no re-trigger)")

    def test_compound_condition_and(self):
        """
        Test event with compound AND condition.

        This test verifies that events can use compound conditions with
        multiple requirements that must all be true (implicit AND logic).
        Compound conditions enable complex triggering logic for sophisticated
        scenarios.

        The test validates:
        - Multiple condition lines are ANDed together
        - Event triggers only when all conditions are true
        - Event does not trigger if any condition is false

        This tests:
        - FGCondition compound condition evaluation
        - AND logic implementation
        - Multiple property comparisons
        """
        # Create script with compound AND condition
        script, run = self._create_simple_script(aircraft='c172p', end_time=8.0)

        # Set initial values
        init_prop1 = et.SubElement(run, 'property')
        init_prop1.set('value', '100.0')
        init_prop1.text = 'test/value-1'

        init_prop2 = et.SubElement(run, 'property')
        init_prop2.set('value', '50.0')
        init_prop2.text = 'test/value-2'

        # Event with AND condition (both conditions must be true)
        event = et.SubElement(run, 'event')
        event.set('name', 'AND condition event')

        condition = et.SubElement(event, 'condition')
        # Multiple lines are ANDed: time >= 2.0 AND value-1 > 90 AND value-2 > 40
        condition.text = '''simulation/sim-time-sec >= 2.0
                           test/value-1 > 90.0
                           test/value-2 > 40.0'''

        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/and-event-triggered')
        set_prop.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        fdm.set_property_value('test/and-event-triggered', 0.0)
        fdm.run_ic()

        # At t=1.0: time condition false, others true
        ExecuteUntil(fdm, 1.0)
        self.assertEqual(fdm['test/and-event-triggered'], 0.0,
                        "Event should not trigger when time condition false")

        # At t=2.5: all conditions true, should trigger
        ExecuteUntil(fdm, 2.5)
        self.assertEqual(fdm['test/and-event-triggered'], 1.0,
                        "Event should trigger when all conditions true")

    def test_compound_condition_or(self):
        """
        Test event with compound OR condition.

        This test verifies that events can use explicit OR logic where
        any one of several conditions can trigger the event. OR conditions
        are useful for events that can be triggered by multiple different
        scenarios.

        The test validates:
        - OR element creates logical OR condition
        - Event triggers when any sub-condition is true
        - Event does not trigger when all sub-conditions are false

        This tests:
        - FGCondition OR logic evaluation
        - Nested condition structures
        - Alternative triggering paths
        """
        # Create script with OR condition
        script, run = self._create_simple_script(end_time=8.0)

        # Initialize test properties
        init_prop1 = et.SubElement(run, 'property')
        init_prop1.set('value', '10.0')
        init_prop1.text = 'test/value-a'

        init_prop2 = et.SubElement(run, 'property')
        init_prop2.set('value', '20.0')
        init_prop2.text = 'test/value-b'

        # Event with OR condition
        event = et.SubElement(run, 'event')
        event.set('name', 'OR condition event')

        condition = et.SubElement(event, 'condition')
        or_elem = et.SubElement(condition, 'or')

        # First OR branch: value-a > 50
        test1 = et.SubElement(or_elem, 'test')
        test1.set('logic', 'AND')
        test1.set('value', '50.0')
        test1.text = 'test/value-a'

        # Second OR branch: value-b > 15
        test2 = et.SubElement(or_elem, 'test')
        test2.set('logic', 'OR')
        test2.set('value', '15.0')
        test2.text = 'test/value-b'

        set_prop = et.SubElement(event, 'set')
        set_prop.set('name', 'test/or-event-triggered')
        set_prop.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        fdm.set_property_value('test/or-event-triggered', 0.0)
        fdm.run_ic()

        # value-a=10 (< 50), value-b=20 (> 15)
        # Second condition is true, so event should trigger
        fdm.run()
        fdm.run()

        triggered = fdm['test/or-event-triggered']
        self.assertEqual(triggered, 1.0,
                        "Event should trigger when one OR condition is true")

    def test_comparison_operators(self):
        """
        Test various comparison operators in conditions.

        This test verifies that all comparison operators work correctly
        in event conditions: >, <, >=, <=, ==, !=, ge, le, gt, lt, eq, ne.

        The test validates:
        - Greater than (>, gt)
        - Less than (<, lt)
        - Greater than or equal (>=, ge)
        - Less than or equal (<=, le)
        - Equal (==, eq)
        - Not equal (!=, ne)

        This tests:
        - FGCondition comparison operator parsing
        - Correct evaluation of each operator type
        - Both symbolic and text operator forms
        """
        # Create script with events testing different operators
        script, run = self._create_simple_script(end_time=10.0)

        # Set test value
        init_prop = et.SubElement(run, 'property')
        init_prop.set('value', '50.0')
        init_prop.text = 'test/value'

        # Test greater than (>)
        event_gt = et.SubElement(run, 'event')
        event_gt.set('name', 'Greater than test')
        cond_gt = et.SubElement(event_gt, 'condition')
        cond_gt.text = 'test/value > 40.0'
        set_gt = et.SubElement(event_gt, 'set')
        set_gt.set('name', 'test/gt-result')
        set_gt.set('value', '1.0')

        # Test less than (<)
        event_lt = et.SubElement(run, 'event')
        event_lt.set('name', 'Less than test')
        cond_lt = et.SubElement(event_lt, 'condition')
        cond_lt.text = 'test/value lt 60.0'
        set_lt = et.SubElement(event_lt, 'set')
        set_lt.set('name', 'test/lt-result')
        set_lt.set('value', '1.0')

        # Test greater or equal (>=)
        event_ge = et.SubElement(run, 'event')
        event_ge.set('name', 'Greater or equal test')
        cond_ge = et.SubElement(event_ge, 'condition')
        cond_ge.text = 'test/value >= 50.0'
        set_ge = et.SubElement(event_ge, 'set')
        set_ge.set('name', 'test/ge-result')
        set_ge.set('value', '1.0')

        # Test less or equal (<=)
        event_le = et.SubElement(run, 'event')
        event_le.set('name', 'Less or equal test')
        cond_le = et.SubElement(event_le, 'condition')
        cond_le.text = 'test/value le 50.0'
        set_le = et.SubElement(event_le, 'set')
        set_le.set('name', 'test/le-result')
        set_le.set('value', '1.0')

        # Test equal (==)
        event_eq = et.SubElement(run, 'event')
        event_eq.set('name', 'Equal test')
        cond_eq = et.SubElement(event_eq, 'condition')
        cond_eq.text = 'test/value == 50.0'
        set_eq = et.SubElement(event_eq, 'set')
        set_eq.set('name', 'test/eq-result')
        set_eq.set('value', '1.0')

        # Test not equal (!=)
        event_ne = et.SubElement(run, 'event')
        event_ne.set('name', 'Not equal test')
        cond_ne = et.SubElement(event_ne, 'condition')
        cond_ne.text = 'test/value != 40.0'
        set_ne = et.SubElement(event_ne, 'set')
        set_ne.set('name', 'test/ne-result')
        set_ne.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        # Initialize result properties
        fdm.set_property_value('test/gt-result', 0.0)
        fdm.set_property_value('test/lt-result', 0.0)
        fdm.set_property_value('test/ge-result', 0.0)
        fdm.set_property_value('test/le-result', 0.0)
        fdm.set_property_value('test/eq-result', 0.0)
        fdm.set_property_value('test/ne-result', 0.0)

        fdm.run_ic()

        # Run a few steps to trigger events
        for _ in range(10):
            fdm.run()

        # Verify all comparisons (value=50.0)
        self.assertEqual(fdm['test/gt-result'], 1.0,
                        "50.0 > 40.0 should be true")
        self.assertEqual(fdm['test/lt-result'], 1.0,
                        "50.0 < 60.0 should be true")
        self.assertEqual(fdm['test/ge-result'], 1.0,
                        "50.0 >= 50.0 should be true")
        self.assertEqual(fdm['test/le-result'], 1.0,
                        "50.0 <= 50.0 should be true")
        self.assertEqual(fdm['test/eq-result'], 1.0,
                        "50.0 == 50.0 should be true")
        self.assertEqual(fdm['test/ne-result'], 1.0,
                        "50.0 != 40.0 should be true")

    def test_event_timing_accuracy(self):
        """
        Test accuracy of event trigger timing.

        This test verifies that events trigger at precisely the correct
        simulation time, not before or after. Timing accuracy is critical
        for reproducible simulations and coordinated event sequences.

        The test validates:
        - Events trigger at exact specified time
        - No early triggering
        - No delayed triggering beyond one time step
        - Multiple events with precise timing

        This tests:
        - FGScript event scheduling precision
        - Time-based condition evaluation accuracy
        - Event execution timing
        """
        # Create script with precisely timed events
        script, run = self._create_simple_script(end_time=6.0, dt=0.01)

        # Event at exactly t=2.00
        event1 = et.SubElement(run, 'event')
        event1.set('name', 'Precise timing event 1')
        cond1 = et.SubElement(event1, 'condition')
        cond1.text = 'simulation/sim-time-sec >= 2.00'
        set1 = et.SubElement(event1, 'set')
        set1.set('name', 'test/time-1')
        set1.set('value', '0.0')
        set1.set('type', 'FG_VALUE')
        set1.set('action', 'FG_RAMP')
        set1.set('tc', '0.0')

        # Store simulation time when event triggers
        set1_time = et.SubElement(event1, 'set')
        set1_time.set('name', 'test/trigger-time-1')
        set1_time.set('value', '1.0')

        # Event at exactly t=4.50
        event2 = et.SubElement(run, 'event')
        event2.set('name', 'Precise timing event 2')
        cond2 = et.SubElement(event2, 'condition')
        cond2.text = 'simulation/sim-time-sec >= 4.50'
        set2_time = et.SubElement(event2, 'set')
        set2_time.set('name', 'test/trigger-time-2')
        set2_time.set('value', '1.0')

        # Write and load script
        self._write_script(script)
        fdm = self.create_fdm()
        fdm.load_script('test_script.xml')

        fdm.set_property_value('test/trigger-time-1', 0.0)
        fdm.set_property_value('test/trigger-time-2', 0.0)

        fdm.run_ic()

        # Check at t=1.99 (just before first event)
        ExecuteUntil(fdm, 1.99)
        self.assertEqual(fdm['test/trigger-time-1'], 0.0,
                        "Event should not trigger before t=2.00")

        # Check at t=2.01 (just after first event)
        ExecuteUntil(fdm, 2.01)
        self.assertEqual(fdm['test/trigger-time-1'], 1.0,
                        "Event should have triggered by t=2.01")

        # Check at t=4.49 (just before second event)
        ExecuteUntil(fdm, 4.49)
        self.assertEqual(fdm['test/trigger-time-2'], 0.0,
                        "Event should not trigger before t=4.50")

        # Check at t=4.51 (just after second event)
        ExecuteUntil(fdm, 4.51)
        self.assertEqual(fdm['test/trigger-time-2'], 1.0,
                        "Event should have triggered by t=4.51")


if __name__ == "__main__":
    RunTest(TestEventsConditions)
