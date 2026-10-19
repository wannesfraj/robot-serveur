"""Cafe-like scenarios: wide static obstacles, chairs beside the robot, A -> B -> base.

Same closed loop as test_obstacle_recovery_flow (real safety / local / global nodes,
ideal kinematics, ray-cast LiDAR in the Webots angle convention).

Run: python -m unittest discover -s tests -p "test_*.py" -v
"""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
from test_obstacle_recovery_flow import World  # noqa: E402


def table(cx, cy, half_len, along='y', r=0.12, step=0.15):
    """A wide obstacle (table, counter, parked trolley) made of small circles."""
    n = int(round(2*half_len/step))
    pts = []
    for k in range(n+1):
        offset = -half_len+k*step
        pts.append((cx, cy+offset, r) if along == 'y' else (cx+offset, cy, r))
    return [lambda t, p=p: p for p in pts]


def chair(x, y, r=0.18):
    return [lambda t: (x, y, r)]


def touched(world, obstacles_fn, body=0.25):
    """Smallest distance between robot centre and any obstacle surface."""
    best = math.inf
    for _, x, y, _ in world.trace:
        for f in obstacles_fn:
            ox, oy, r = f(0.0)
            best = min(best, math.hypot(x-ox, y-oy)-r)
    return best


def reached(world, goal, tol=0.15):
    return math.hypot(world.x-goal[0], world.y-goal[1]) < tol


def summary(world):
    rec = [json.loads(m.data).get('recovery', {}) for m in world.local.status.messages]
    attempts = max([r.get('attempts', 0) for r in rec] or [0])
    last = json.loads(world.local.status.messages[-1].data) if world.local.status.messages else {}
    return f'pos=({world.x:.2f},{world.y:.2f}) state={last.get("state")} reason={last.get("reason")} attempts={attempts}'


class CafeScenarios(unittest.TestCase):
    def run_goal(self, world, goal, seconds):
        world.go(*goal)
        world.run(seconds, until=lambda w: reached(w, goal) and 'goal_reached' in w.states()[-3:])
        return reached(world, goal)

    def test_wide_table_not_in_map(self):
        """A 1.2 m wide table appears on the straight route; the map never contains it."""
        obstacles = table(3.0, 0.0, 0.6)
        world = World(obstacles, known_from=-1.0)
        ok = self.run_goal(world, (6.0, 0.0), 180)
        self.assertTrue(ok, summary(world))
        self.assertGreater(touched(world, obstacles), 0.30)

    def test_wide_table_with_chairs_beside_stop_point(self):
        """Chairs on both sides of the stop position: the robot cannot rotate where it stops."""
        obstacles = table(3.0, 0.0, 0.7) + chair(2.45, 0.62) + chair(2.45, -0.62)
        world = World(obstacles, known_from=-1.0)
        ok = self.run_goal(world, (6.0, 0.0), 240)
        self.assertTrue(ok, summary(world))
        self.assertGreater(touched(world, obstacles), 0.30)

    def test_table_mapped_late_by_mapping_node(self):
        """The mapping node adds the obstacle to the map once it is within 1.2 m."""
        obstacles = table(3.0, 0.0, 0.6) + chair(2.4, 0.75)
        world = World(obstacles, known_from=1.2)
        ok = self.run_goal(world, (6.0, 0.0), 180)
        self.assertTrue(ok, summary(world))
        self.assertGreater(touched(world, obstacles), 0.30)

    def test_service_mission_a_then_b_then_base(self):
        """Serve table A, then table B, then come back to base; a counter blocks the way."""
        obstacles = table(2.5, 0.0, 0.7) + chair(1.95, 0.6) + table(2.5, 3.0, 0.5, along='x')
        world = World(obstacles, known_from=-1.0)
        for goal in ((5.0, 0.5), (5.0, 3.2), (0.0, 0.0)):
            ok = self.run_goal(world, goal, 240)
            self.assertTrue(ok, f'goal {goal}: '+summary(world))
        self.assertGreater(touched(world, obstacles), 0.30)


    def test_unreachable_goal_fails_cleanly_instead_of_looping(self):
        """The target table is fully surrounded: after bounded attempts the goal fails with a reason."""
        ring = []
        for k in range(28):
            a = 2*math.pi*k/28
            ring.append((lambda t, p=(4.5+0.9*math.cos(a), 0.9*math.sin(a), 0.12): p))
        world = World(ring, known_from=-1.0)
        world.go(4.5, 0.0)
        world.run(400, until=lambda w: 'goal_failed' in w.states()[-3:])
        last = json.loads(world.local.status.messages[-1].data)
        self.assertEqual(last['state'], 'goal_failed', summary(world))
        self.assertIn('obstacle_unavoidable', last['reason'])
        self.assertGreater(touched(world, ring), 0.30)

    def test_static_obstacle_removed_later_is_forgotten(self):
        """A trolley is parked on the route (static), then a waiter moves it away.

        The global planner must forget it as soon as the LiDAR sees through it,
        so the next trip uses the direct route again."""
        trolley = table(3.0, 0.0, 0.5)
        moved_at = [None]

        def present(f):
            return lambda t: None if moved_at[0] is not None and t >= moved_at[0] else f(t)
        world = World([present(f) for f in trolley], known_from=-1.0)
        ok = self.run_goal(world, (6.0, 0.0), 200)
        self.assertTrue(ok, summary(world))
        self.assertGreater(len(world.global_planner.transient_obstacles), 0)
        moved_at[0] = world.t
        ok = self.run_goal(world, (0.0, 0.0), 200)
        self.assertTrue(ok, summary(world))
        # straight return: never more than 0.4 m off the x axis once the trolley is gone
        back = [row for row in world.trace if row[0] > moved_at[0]+1.0]
        self.assertLess(max(abs(row[2]) for row in back), 0.4)


if __name__ == '__main__':
    unittest.main()
