"""Closed loop with the production nodes (safety supervisor, local planner, global A*).

Only ROS transport, wheel dynamics and the LiDAR are substituted: ideal kinematics,
ray-cast scan against circular obstacles. This is NOT Webots physics or DDS.

Run: python -m unittest discover -s tests -p test_obstacle_recovery_flow.py -v
"""
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'tests'))
for package in (ROOT/'ros_ws/src').iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))
import numpy  # noqa: F401  (import before sys.modules is patched, numpy cannot be re-imported)
import ros_sim
from types import ModuleType as _ModuleType
# The archive does not ship the EKF / map-localizer modules imported by
# fabtino_core.localization.__init__. Provide empty stand-ins only when missing.
for _name in ('ekf', 'motion_model', 'map_localizer'):
    _full = 'fabtino_core.localization.'+_name
    try:
        __import__(_full)
    except ImportError:
        _stub = _ModuleType(_full)
        for _attr in ('LocalizationEKF', 'EKFConfig', 'wrap_angle', 'KnownMapLocalizer', 'KnownMapLocalizerConfig'):
            setattr(_stub, _attr, None)
        sys.modules[_full] = _stub

RADIUS, RES = 8.0, 0.1
_NODES = {}


def node_classes():
    if not _NODES:
        import importlib
        with patch.dict(sys.modules):
            ros_sim.install()
            for key, (module, name) in dict(
                    safety=('fabtino_safety.safety_supervisor_node', 'SafetySupervisor'),
                    local=('fabtino_navigation.local_planner_node', 'LocalPlannerNode'),
                    global_planner=('fabtino_navigation.global_planner_node', 'GlobalPlannerNode')).items():
                _NODES[key] = getattr(importlib.import_module(module), name)
    return _NODES


class World:
    """Robot + obstacles + the sensor/map messages the real stack would provide."""

    def __init__(self, obstacles, known_from=1.2, soc=None, reveal_after_block_s=None,
                 webots_scan=True, start=(0.0, 0.0, 0.0)):
        # Webots publishes the Pioneer scan with +angle towards the RIGHT
        # (see fabtino_core.localization.frames, lidar_angle_sign=-1).
        self.scan_sign = -1.0 if webots_scan else 1.0
        self.bus = ros_sim.Bus()
        nodes = node_classes()
        self.safety = nodes['safety']()
        self.local = nodes['local']()
        self.global_planner = nodes['global_planner']()
        self.global_planner.parameters['battery_required'] = soc is not None
        self.obstacles = obstacles            # list of callables t -> (x, y, r) or None
        self.known_from = known_from          # obstacle enters the A* map once this close
        self.known = set()
        self.cached = {}
        self.reveal_after_block_s = reveal_after_block_s   # A* only learns the obstacle this long after the stop
        self.first_block = None
        self.x, self.y, self.yaw = start
        self.v = self.w = 0.0
        self.trace = []
        self.pub = {name: ros_sim.Publisher(self.bus, topic) for name, topic in dict(
            odom='/fabtino/odometry/filtered', scan='/fabtino/scan',
            map='/fabtino/navigation_map', goal='/navigation/goal',
            battery='/fabtino/battery').items()}
        self.soc = soc
        self.publish_map()

    @property
    def t(self):
        return self.bus.now_ns*1e-9

    def active_obstacles(self):
        return [o for o in (f(self.t) for f in self.obstacles) if o is not None]

    def publish_map(self):
        size = int(2*RADIUS/RES)
        data = [0]*(size*size)
        for index in self.known:
            ox, oy, r = self.cached[index]
            for gy in range(size):
                y = -RADIUS+(gy+.5)*RES
                if abs(y-oy) > r+RES:
                    continue
                for gx in range(size):
                    x = -RADIUS+(gx+.5)*RES
                    if math.hypot(x-ox, y-oy) <= r+RES*.5:
                        data[gy*size+gx] = 100
        message = ros_sim.OccupancyGrid()
        message.header.stamp = ros_sim.stamp(self.bus.now_ns)
        message.info.resolution = RES
        message.info.width = message.info.height = size
        message.info.origin.position.x = message.info.origin.position.y = -RADIUS
        message.data = data
        self.pub['map'].publish(message)

    def publish_sensors(self):
        stamp = ros_sim.stamp(self.bus.now_ns)
        odom = ros_sim.Odometry()
        odom.header.stamp = stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        self.pub['odom'].publish(odom)
        scan = ros_sim.LaserScan()
        scan.header.stamp = stamp
        scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
        ranges = []
        for i in range(360):
            angle = self.scan_sign*(scan.angle_min+i*scan.angle_increment)+self.yaw
            best = float('inf')
            for ox, oy, r in self.active_obstacles():
                dx, dy = ox-self.x, oy-self.y
                proj = dx*math.cos(angle)+dy*math.sin(angle)
                disc = r*r-(dx*dx+dy*dy-proj*proj)
                if proj > 0 and disc >= 0:
                    best = min(best, proj-math.sqrt(disc))
            ranges.append(best)
        scan.ranges = ranges
        self.pub['scan'].publish(scan)
        changed = False
        reason = json.loads(self.safety.status.messages[-1].data)['reason'] if self.safety.status.messages else ''
        if self.first_block is None and reason.startswith('front_obstacle'):
            self.first_block = self.t
        revealed = (self.reveal_after_block_s is not None and self.first_block is not None
                    and self.t-self.first_block >= self.reveal_after_block_s)
        for ox, oy, r in self.active_obstacles():
            key = (round(ox, 1), round(oy, 1), r)
            near = math.hypot(ox-self.x, oy-self.y) < self.known_from+r
            if (near or revealed) and key not in self.cached.values():
                self.cached[len(self.cached)] = (ox, oy, r)
                self.known.add(len(self.cached)-1)
                changed = True
        if changed:
            self.publish_map()
        if self.soc is not None:
            battery = ros_sim.String()
            battery.data = json.dumps(dict(soc_pct=self.soc, config=dict(
                capacity_wh=100.0, wh_per_m=0.10, wh_per_rad=0.02, idle_w=20.0, avg_speed_mps=0.2,
                reserve_pct=15.0, warning_pct=20.0, safety_factor=1.25, resample_m=1.0, turn_deadband_rad=0.25)))
            self.pub['battery'].publish(battery)

    def go(self, x, y):
        from fabtino_navigation.goal_control import Goal
        message = ros_sim.String()
        message.data = json.dumps(Goal.from_request({'x': x, 'y': y}).payload())
        self.pub['goal'].publish(message)

    def run(self, seconds, until=None, dt=0.02):
        end = self.t+seconds
        while self.t < end:
            self.bus.tick(dt)
            command = self.safety.pub.messages[-1] if self.safety.pub.messages else None
            self.v, self.w = (command.linear.x, command.angular.z) if command else (0., 0.)
            self.x += self.v*math.cos(self.yaw)*dt
            self.y += self.v*math.sin(self.yaw)*dt
            self.yaw = (self.yaw+self.w*dt+math.pi) % (2*math.pi)-math.pi
            self.publish_sensors()
            states = self.local.status.messages[-1].data if self.local.status.messages else '{}'
            self.trace.append((self.t, self.x, self.y, json.loads(states).get('state')))
            if until and until(self):
                return True
        return False

    def states(self):
        return [json.loads(m.data).get('state') for m in self.local.status.messages]

    def global_states(self):
        return [json.loads(m.data).get('state') for m in self.global_planner.status.messages]


class RecoveryFlowTests(unittest.TestCase):
    def test_static_obstacle_robot_waits_backs_off_replans_and_arrives(self):
        world = World([lambda t: (3.0, 0.0, 0.25)], known_from=-1.0, reveal_after_block_s=3.0)
        world.go(6.0, 0.0)
        reached = world.run(90, until=lambda w: 'goal_reached' in w.states())
        states = world.states()
        self.assertIn('recovering', states, 'robot never entered recovery')
        self.assertTrue(reached, f'goal not reached, last={world.trace[-1]}')
        recovering = [row for row in world.trace if row[3] == 'recovering']
        # it held still about 3.5 s before moving back
        first = recovering[0]
        moved_back = next(row for row in recovering if row[1] < first[1]-0.02)
        self.assertGreaterEqual(moved_back[0]-first[0], 3.4)
        self.assertLess(moved_back[0]-first[0], 5.0)
        # backed off a bounded amount
        self.assertGreater(min(row[1] for row in recovering), first[1]-0.65)
        # never touched the obstacle
        self.assertGreater(min(math.hypot(row[1]-3.0, row[2]) for row in world.trace), 0.25+0.2)
        self.assertLess(math.hypot(world.x-6.0, world.y), 0.15)

    def test_dynamic_obstacle_robot_only_waits_then_resumes(self):
        # an object stands in the way for 1.5 s and then walks off
        world = World([lambda t: (3.0, 0.0, 0.25) if t < 11.5 else None], known_from=0.0)
        world.go(6.0, 0.0)
        start = world.t
        reached = world.run(60, until=lambda w: 'goal_reached' in w.states())
        self.assertTrue(reached)
        self.assertNotIn('backing_off', [m for m in
                         (json.loads(x.data).get('recovery', {}).get('state') for x in world.local.status.messages)])
        xs = [row[1] for row in world.trace]
        # x never decreases by more than a hair (no back-off), it just paused
        self.assertTrue(all(b >= a-0.02 for a, b in zip(xs, xs[1:])))
        self.assertGreater(start, 0)

    def rotation_blocked_world(self, recovery):
        # Goal is behind the robot (needs a turn) and an object sits 0.46 m from its side:
        # the supervisor forbids rotating within 0.52 m, so without recovery the robot freezes.
        world = World([lambda t: (0.3, 0.45, 0.08)], known_from=5.0)
        world.local.parameters['recovery_enabled'] = recovery
        world.go(-4.0, 0.0)
        return world

    def test_without_recovery_robot_freezes_when_it_cannot_rotate(self):
        world = self.rotation_blocked_world(False)
        world.run(8)
        self.assertEqual(world.states()[-1], 'turning_to_path')
        self.assertLess(math.hypot(world.x, world.y), 0.01)
        self.assertAlmostEqual(world.yaw, 0.0, places=2)

    def test_rotation_blocked_robot_waits_backs_off_then_turns_and_follows_path(self):
        world = self.rotation_blocked_world(True)
        turned = world.run(25, until=lambda w: abs(w.yaw) > 1.5)
        self.assertTrue(turned, f'never rotated, last={world.trace[-1]}')
        recovering = [row for row in world.trace if row[3] == 'recovering']
        held = next(row for row in recovering if row[1] < -0.02)
        self.assertGreaterEqual(held[0]-recovering[0][0], 3.4)           # waited ~3.5 s first
        self.assertLess(min(row[1] for row in world.trace), -0.2)        # then backed away
        self.assertGreater(min(row[1] for row in world.trace), -0.7)     # but only a little

    def test_obstacle_missing_from_map_is_added_to_a_star_and_goal_is_reached(self):
        # The map never contains the obstacle; the safety LiDAR must add it to A*.
        world = World([lambda t: (3.0, 0.0, 0.25)], known_from=-1.0)
        world.go(6.0, 0.0)
        reached = world.run(150, until=lambda w: 'goal_reached' in w.states())
        self.assertTrue(reached, f'last={world.trace[-1]}')
        self.assertIn('recovering', world.states())
        # it detoured and reached the target without touching the obstacle
        self.assertGreater(min(math.hypot(row[1]-3.0, row[2]) for row in world.trace), 0.25+0.2)
        self.assertLess(math.hypot(world.x-6.0, world.y), 0.15)


class BatteryFlowTests(unittest.TestCase):
    def test_trajectory_refused_when_battery_would_cross_threshold(self):
        world = World([], soc=15.5)
        world.go(6.0, 0.0)
        world.run(3)
        self.assertIn('battery_insufficient', world.global_states())
        refusal = next(json.loads(m.data) for m in world.global_planner.status.messages
                       if json.loads(m.data)['state'] == 'battery_insufficient')
        self.assertIn('Battery too low', refusal['reason'])
        self.assertEqual(world.global_planner.path, [])
        self.assertLess(math.hypot(world.x, world.y), 0.05)             # robot never moved

    def test_trip_ending_between_15_and_20_is_accepted_with_warning(self):
        world = World([], soc=20.5)
        world.go(6.0, 0.0)
        world.run(3)
        self.assertIn('path_ready', world.global_states())
        report = next(json.loads(m.data)['battery'] for m in world.global_planner.status.messages
                      if json.loads(m.data)['state'] == 'path_ready')
        self.assertTrue(report['accepted'])
        self.assertTrue(report['warning'])
        self.assertGreaterEqual(report['soc_after_pct'], 15.0)
        self.assertLessEqual(report['soc_after_pct'], 20.0)

    def test_trajectory_accepted_with_enough_battery(self):
        world = World([], soc=90.0)
        world.go(6.0, 0.0)
        world.run(3)
        self.assertIn('path_ready', world.global_states())
        self.assertNotIn('battery_insufficient', world.global_states())
        report = json.loads(world.global_planner.status.messages[-1].data)['battery']
        self.assertTrue(report['accepted'])
        remaining = math.hypot(6.0-world.x, world.y)
        self.assertAlmostEqual(report['distance_m'], remaining, delta=0.2)


if __name__ == '__main__':
    unittest.main()
