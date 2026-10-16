"""Run without ROS: python -m unittest discover -s tests -p test_recovery_battery.py"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'ros_ws/src/fabtino_navigation'))
sys.path.insert(0, str(ROOT/'ros_ws/src/fabtino_core'))
from fabtino_navigation.obstacle_recovery import ObstacleRecovery
from fabtino_core.battery import BatteryConfig, BatteryState, estimate_path_cost, evaluate_trip


def safety(reason='ok', front=None, rear=None, left=None, right=None, turn=True, nearest=None, bearing=0.0):
    return dict(reason=reason, front_m=front, rear_m=rear, left_m=left, right_m=right,
                turn_allowed=turn, front_stop_m=0.60, rear_stop_m=0.65, rotation_clearance_m=0.52,
                nearest_m=nearest, nearest_bearing_rad=bearing)


BLOCKED = safety('front_obstacle=0.60m', front=0.60, nearest=0.60)


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.r = ObstacleRecovery()
        self.pose = (0.0, 0.0, 0.0)

    def run_until(self, start, stop, safety_msg, pose=None, path_ready=True, dt=0.05):
        t, out = start, None
        while t < stop:
            out = self.r.update(t, pose or self.pose, safety_msg, True, path_ready)
            t += dt
        return out

    def test_robot_holds_then_resumes_when_obstacle_is_dynamic(self):
        self.run_until(0.0, 1.5, BLOCKED)
        self.assertEqual(self.r.state, ObstacleRecovery.WAITING)
        # obstacle walks away after 1.5 s: no manoeuvre, follower drives again
        out = self.run_until(1.5, 2.5, safety())
        self.assertIsNone(out)
        self.assertEqual(self.r.state, ObstacleRecovery.IDLE)
        self.assertEqual(self.r.attempts, 0)

    def test_static_obstacle_triggers_back_off_after_wait(self):
        self.run_until(0.0, 3.4, BLOCKED)
        self.assertEqual(self.r.state, ObstacleRecovery.WAITING)          # still waiting at 3.4 s
        out = self.run_until(3.4, 3.8, BLOCKED)
        self.assertEqual(self.r.state, ObstacleRecovery.BACKING)
        self.assertLess(out[0], 0.0)

    def test_back_off_stops_when_clear_then_replans_and_resumes(self):
        self.run_until(0.0, 3.6, BLOCKED)
        pose = (-0.2, 0.0, 0.0)
        self.r.update(3.7, pose, safety(front=0.95, nearest=0.95), True, True)
        self.assertEqual(self.r.state, ObstacleRecovery.REPLANNING)
        self.assertTrue(self.r.take_replan_request())
        self.assertEqual(self.r.update(3.8, pose, safety(front=0.95), True, False), (0.0, 0.0))
        self.assertIsNone(self.r.update(4.3, pose, safety(front=0.95), True, True))
        self.assertEqual(self.r.state, ObstacleRecovery.IDLE)

    def test_back_off_distance_is_capped_then_rotates_if_still_blocked(self):
        self.run_until(0.0, 3.6, BLOCKED)
        self.r.update(4.0, (-0.55, 0.0, 0.0), BLOCKED, True, True)
        self.assertEqual(self.r.state, ObstacleRecovery.ROTATING)

    def test_back_off_distance_is_capped_then_replans_if_rotation_not_allowed(self):
        no_turn = safety('front_obstacle=0.60m', front=0.60, turn=False, nearest=0.6)
        self.run_until(0.0, 3.6, no_turn)
        self.r.update(4.0, (-0.55, 0.0, 0.0), no_turn, True, True)
        self.assertEqual(self.r.state, ObstacleRecovery.REPLANNING)

    def test_rotates_towards_free_side_when_rear_is_blocked(self):
        blocked_rear = safety('front_obstacle=0.60m', front=0.60, rear=0.40, left=0.5, right=2.0, nearest=0.4)
        out = self.run_until(0.0, 3.7, blocked_rear)
        self.assertEqual(self.r.state, ObstacleRecovery.ROTATING)
        self.assertEqual(out[0], 0.0)
        self.assertLess(out[1], 0.0)                                      # right side has more room

    def test_trapped_robot_holds_and_keeps_goal_active(self):
        trapped = safety('front_obstacle=0.60m', front=0.60, rear=0.40, turn=False, nearest=0.4)
        out = self.run_until(0.0, 3.7, trapped)
        self.assertEqual(self.r.state, ObstacleRecovery.WAITING)
        self.assertTrue(self.r.active)
        self.assertIsNone(self.r.failure)
        self.assertEqual(out, (0.0, 0.0))

    def test_retries_beyond_three_attempts_without_abandoning_goal(self):
        t = 0.0
        for _ in range(5):
            self.run_until(t, t+3.6, BLOCKED)
            self.r.update(t+3.7, (-0.2, 0, 0), safety(front=0.95), True, True)   # backed off -> replanning
            self.r.take_replan_request()
            self.r.update(t+4.2, (-0.2, 0, 0), safety(front=0.95), True, True)   # route back -> idle
            t += 4.5
        self.assertGreater(self.r.attempts, 3)
        self.assertIsNone(self.r.failure)

    def test_no_route_after_recovery_triggers_another_safe_attempt(self):
        self.run_until(0.0, 3.6, BLOCKED)
        self.r.update(3.7, (-0.2, 0, 0), safety(front=0.95), True, True)
        out = self.r.update(14.0, (-0.2, 0, 0), safety(front=0.95), True, False)
        self.assertEqual(self.r.state, ObstacleRecovery.BACKING)
        self.assertLess(out[0], 0.0)
        self.assertIsNone(self.r.failure)

    def test_inactive_goal_aborts_manoeuvre(self):
        self.run_until(0.0, 3.6, BLOCKED)
        self.assertTrue(self.r.active)
        self.assertIsNone(self.r.update(3.7, self.pose, BLOCKED, False, False))
        self.assertEqual(self.r.state, ObstacleRecovery.IDLE)


class BatteryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = BatteryConfig(capacity_wh=100.0, reserve_pct=15.0, warning_pct=20.0, safety_factor=1.0)
        self.line = [(float(x), 0.0) for x in range(0, 41)]               # 40 m straight

    def test_cost_grows_with_distance_and_turning(self):
        straight = estimate_path_cost(self.line, self.cfg)
        l_shape = estimate_path_cost([(0, 0), (20, 0), (20, 20)], self.cfg)
        self.assertAlmostEqual(straight['distance_m'], 40.0)
        self.assertAlmostEqual(l_shape['turn_rad'], math.pi/2, places=3)
        self.assertGreater(l_shape['energy_wh'], estimate_path_cost([(0, 0), (40, 0)], self.cfg)['energy_wh'])

    def test_grid_zigzag_is_not_counted_as_turning(self):
        zigzag = [(i*0.1, (0.1 if i % 2 else 0.0)) for i in range(60)]
        self.assertLess(estimate_path_cost(zigzag, self.cfg)['turn_rad'], 0.5)

    def test_accepts_when_reserve_is_kept(self):
        verdict = evaluate_trip(self.line, soc_pct=90.0, config=self.cfg)
        self.assertTrue(verdict['accepted'])
        self.assertGreaterEqual(verdict['soc_after_pct'], 15.0)

    def test_refuses_when_trip_would_cross_the_threshold(self):
        verdict = evaluate_trip(self.line, soc_pct=20.0, config=self.cfg)
        self.assertFalse(verdict['accepted'])
        self.assertIn('Battery too low', verdict['reason'])

    def test_refuses_when_already_below_reserve(self):
        self.assertFalse(evaluate_trip([(0, 0), (0.5, 0)], soc_pct=14.5, config=self.cfg)['accepted'])

    def test_forecast_between_15_and_20_is_accepted_with_warning(self):
        verdict=evaluate_trip(self.line,soc_pct=25.0,config=self.cfg)
        self.assertTrue(verdict['accepted'])
        self.assertTrue(15.0<=verdict['soc_after_pct']<=20.0)
        self.assertTrue(verdict['warning'])
        self.assertIn('recharge',verdict['warning_reason'])

    def test_waiting_at_delivery_stops_is_included_in_energy_cost(self):
        route=evaluate_trip([(0,0),(10,0)],soc_pct=100,config=self.cfg)
        waiting=evaluate_trip([(0,0),(10,0)],soc_pct=100,config=self.cfg,idle_duration_s=3600)
        self.assertEqual(waiting['wait_s'],3600)
        self.assertEqual(waiting['distance_m'],route['distance_m'])
        self.assertGreater(waiting['required_pct'],route['required_pct'])

    def test_safety_factor_makes_the_decision_more_conservative(self):
        tight = BatteryConfig(capacity_wh=100.0, reserve_pct=15.0, safety_factor=1.0)
        careful = BatteryConfig(capacity_wh=100.0, reserve_pct=15.0, safety_factor=3.0)
        base = estimate_path_cost(self.line, tight)['percent']
        soc = 20.0+base*1.5
        self.assertTrue(evaluate_trip(self.line, soc, tight)['accepted'])
        self.assertFalse(evaluate_trip(self.line, soc, careful)['accepted'])

    def test_state_integrates_motion_and_ignores_clock_jumps(self):
        state = BatteryState(self.cfg, 100.0)
        state.update(0.0, 0.0, 0.0)
        for i in range(1, 201):
            state.update(i*0.1, 0.25, 0.0)                                # 20 s at 0.25 m/s
        expected = 0.10*0.25*20+20.0*20/3600
        self.assertAlmostEqual(100.0-state.soc_pct, expected, places=2)
        before = state.soc_pct
        state.update(10_000.0, 0.25, 0.0)                                 # huge gap: capped at 0.5 s
        self.assertLess(before-state.soc_pct, 0.02)   # capped at one 0.5 s step

    def test_set_soc_is_clamped(self):
        state = BatteryState(self.cfg, 50.0)
        state.set_soc(150.0)
        self.assertEqual(state.soc_pct, 100.0)
        state.set_soc(-5.0)
        self.assertEqual(state.soc_pct, 0.0)
        self.assertTrue(state.low)


if __name__ == '__main__':
    unittest.main()
