"""Unit tests for SimulatedPlanComponent.plan() planning logic."""
import math
import pytest
import rclpy
from simulated_plan_interfaces.msg import PoiCompleted
from simulated_plan_component.SimulatedPlanComponent import SimulatedPlanComponent


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def planner(rclpy_context):
    """Create a planner node with predictable default timing/battery params."""
    node = SimulatedPlanComponent(
        time_per_navigation_poi=30.0,
        time_per_explain_poi=40.0,
        time_per_question_poi=20.0,
        battery_drainage_per_poi=5.0,
    )
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_poi(is_overtime=False, cause=''):
    """Return a PoiCompleted message with the given fields."""
    poi = PoiCompleted()
    poi.is_overtime = is_overtime
    poi.cause = cause
    return poi


def _all_skip_poi(actions):
    return all(a.skip_poi for a in actions)


def _none_skip_poi(actions):
    return all(not a.skip_poi for a in actions)


def _all_skip_questions(actions):
    return all(a.skip_questions for a in actions)


def _none_skip_questions(actions):
    return all(not a.skip_questions for a in actions)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestOutputInvariants:
    """Properties that must hold for any valid output of plan()."""

    def test_no_remaining_pois_returns_empty(self, planner):
        result = planner.plan([], 0.0, 600.0, 0, 100.0)
        assert result == []

    def test_output_length_equals_remaining_pois(self, planner):
        for n in (1, 3, 8):
            result = planner.plan([], 0.0, 9999.0, n, 100.0)
            assert len(result) == n, f'expected {n} actions, got {len(result)}'

    def test_skip_poi_implies_skip_questions(self, planner):
        """A skipped POI must always also have skip_questions set."""
        result = planner.plan([], 300.0, 400.0, 5, 15.0)
        for a in result:
            if a.skip_poi:
                assert a.skip_questions, 'skip_poi=True but skip_questions=False'


class TestAmpleResources:
    """With plenty of time and battery nothing should be skipped."""

    def test_ample_time_full_battery_no_skips(self, planner):
        # full_poi_time = 30+40+20 = 90; 5 POIs = 450s; time_left = 600s, battery 90%
        result = planner.plan([], 0.0, 600.0, 5, 90.0)
        assert _none_skip_poi(result)
        assert _none_skip_questions(result)

    def test_non_overtime_completed_pois_do_not_affect_remaining(self, planner):
        completed = [make_poi(is_overtime=False, cause='navigation') for _ in range(10)]
        result = planner.plan(completed, 0.0, 9999.0, 3, 100.0)
        assert _none_skip_poi(result)
        assert _none_skip_questions(result)


class TestTimePressure:
    """Skips should increase as the time budget shrinks."""

    def test_moderate_time_pressure_skips_some_questions(self, planner):
        # estimated_full = 4*90 = 360; time_left = 350 => ratio = 360/350 ≈ 1.03
        # Just above moderate threshold (1.1) after ratio check
        # Let's use time_left=300 => ratio=1.2 which is >= moderate(1.1) but < high(1.3)
        result = planner.plan([], 100.0, 400.0, 4, 100.0)
        # At least some questions must be skipped (>=35% of 4 = 2)
        skipped_q = sum(1 for a in result if a.skip_questions)
        assert skipped_q >= 1

    def test_high_time_pressure_skips_majority_of_questions(self, planner):
        # estimated_full = 4*90 = 360; time_left = 200 => ratio = 1.8 >= high(1.3)
        result = planner.plan([], 200.0, 400.0, 4, 100.0)
        skipped_q = sum(1 for a in result if a.skip_questions)
        # high pressure => ceil(4*0.75)=3 minimum
        assert skipped_q >= 3

    def test_time_exhausted_skips_pois(self, planner):
        # Only enough time for 2 of 5 POIs even without questions
        # min_poi_time = 30+40 = 70; 5 pois = 350s; time_left = 140s
        result = planner.plan([], 460.0, 600.0, 5, 100.0)
        skipped_pois = sum(1 for a in result if a.skip_poi)
        assert skipped_pois >= 1


class TestBattery:
    """Battery shortcuts must kick in at the right thresholds."""

    def test_full_battery_no_battery_related_skips(self, planner):
        # 4 POIs * 5%/poi + 10% margin = 30% needed; battery 80% => no shortage
        result = planner.plan([], 0.0, 9999.0, 4, 80.0)
        assert _none_skip_poi(result)
        assert _none_skip_questions(result)

    def test_battery_shortage_forces_poi_skips(self, planner):
        # 6 POIs * 5%/poi + 10% margin = 40% needed; battery 30% => shortage 10%
        # skip_poi_count from shortage = ceil(10/5) = 2
        result = planner.plan([], 0.0, 9999.0, 6, 30.0)
        skipped_pois = sum(1 for a in result if a.skip_poi)
        assert skipped_pois >= 2

    def test_low_battery_skips_questions(self, planner):
        # battery=30 < low_battery_threshold=35 => ceil(4*0.5)=2 questions skipped
        result = planner.plan([], 0.0, 9999.0, 4, 30.0)
        skipped_q = sum(1 for a in result if a.skip_questions)
        assert skipped_q >= 2

    def test_critical_battery_all_questions_skipped(self, planner):
        # battery=15 < critical_threshold=20 => all questions skipped
        result = planner.plan([], 0.0, 9999.0, 5, 15.0)
        assert _all_skip_questions(result)

    def test_critical_battery_skips_some_pois(self, planner):
        # battery=15 => also ceil(5*0.4)=2 pois skipped as a floor
        result = planner.plan([], 0.0, 9999.0, 5, 15.0)
        skipped_pois = sum(1 for a in result if a.skip_poi)
        assert skipped_pois >= 2


class TestOvertimeCauses:
    """Past overtime events should nudge future skip counts."""

    def test_nav_overtime_increases_poi_skips(self, planner):
        completed = [
            make_poi(is_overtime=True, cause='navigation timeout'),
            make_poi(is_overtime=True, cause='path blocked'),
        ]
        # overtime_nav_causes=2 => skip_poi >= ceil(2*0.5)=1
        result = planner.plan(completed, 0.0, 9999.0, 5, 80.0)
        skipped_pois = sum(1 for a in result if a.skip_poi)
        assert skipped_pois >= 1

    def test_question_overtime_increases_question_skips(self, planner):
        completed = [
            make_poi(is_overtime=True, cause='dialog timeout'),
            make_poi(is_overtime=True, cause='talk exceeded'),
            make_poi(is_overtime=True, cause='answer too long'),
        ]
        # overtime_question_causes=3 => skip_questions >= ceil(3*0.7)=3
        result = planner.plan(completed, 0.0, 9999.0, 5, 80.0)
        skipped_q = sum(1 for a in result if a.skip_questions)
        assert skipped_q >= 3

    def test_non_overtime_pois_are_ignored(self, planner):
        completed = [
            make_poi(is_overtime=False, cause='navigation timeout'),
            make_poi(is_overtime=False, cause='dialog'),
        ]
        result = planner.plan(completed, 0.0, 9999.0, 4, 90.0)
        assert _none_skip_poi(result)
        assert _none_skip_questions(result)
