"""ROS-independent obstacle recovery.

When the safety supervisor stops the robot in front of an obstacle:

1. WAITING   hold position for ``wait_s`` (default 3.5 s). If the obstacle leaves
             (dynamic obstacle that walked past) the robot simply resumes.
2. BACKING   obstacle still there -> it is treated as static. Reverse a little
             (or move forward when the obstacle is behind it) until the safety
             threshold is cleared by ``clear_margin_m``.
   ROTATING  when reversing is impossible (something behind) rotate towards the
             side with more free space until the corridor ahead is free.
3. REPLANNING  drop the old route; the global planner (A*) is already replanning
             every 100 ms from the *current* pose with the obstacle in the map.
             Once a new route arrives the follower continues the mission.

The goal remains active through repeated recovery attempts. Each reverse or turn
is bounded and checked by the safety supervisor; A* keeps looking for a route.
"""
from __future__ import annotations

import math


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ObstacleRecovery:
    IDLE = 'idle'
    WAITING = 'waiting_obstacle'
    BACKING = 'backing_off'
    ROTATING = 'rotating_free'
    REPLANNING = 'replanning'

    def __init__(self, wait_s=3.5, debounce_s=0.4, move_speed_mps=0.10, rotate_rps=0.45,
                 clear_margin_m=0.30, max_move_m=0.50, max_rotate_rad=math.pi/2,
                 phase_timeout_s=8.0, replan_min_s=0.5, replan_timeout_s=10.0,
                 max_attempts=3, attempts_reset_m=1.5):
        self.wait_s = wait_s
        self.debounce_s = debounce_s
        self.move_speed = move_speed_mps
        self.rotate_rps = rotate_rps
        self.clear_margin = clear_margin_m
        self.max_move = max_move_m
        self.max_rotate = max_rotate_rad
        self.phase_timeout = phase_timeout_s
        self.replan_min = replan_min_s
        self.replan_timeout = replan_timeout_s
        self.max_attempts = max_attempts
        self.attempts_reset = attempts_reset_m
        self.attempts = 0
        self.attempt_origin = None
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        """New goal, cancelled goal or finished goal: forget everything."""
        self.state = self.IDLE
        self.kind = None
        self.blocked_since = self.last_blocked = None
        self.phase_started = None
        self.start_pose = None
        self.direction = 0
        self.rotate_sign = 1
        self.rotated = 0.0
        self.last_yaw = None
        self.replan_started = None
        self.replan_requested = False
        self.failure = None
        self.reason = ''
        self.attempts = 0
        self.attempt_origin = None

    @property
    def active(self):
        return self.state in (self.WAITING, self.BACKING, self.ROTATING, self.REPLANNING)

    def take_replan_request(self):
        requested, self.replan_requested = self.replan_requested, False
        return requested

    def snapshot(self):
        return dict(state=self.state, kind=self.kind, attempts=self.attempts, reason=self.reason)

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _blocked_kind(safety):
        reason = str(safety.get('reason', ''))
        if reason.startswith('front_obstacle'):
            return 'front'
        if reason.startswith('rotation_obstacle'):
            return 'rotation'
        return None

    def _cleared(self, safety):
        if self.kind == 'rotation':
            nearest = safety.get('nearest_m')
            clearance = float(safety.get('rotation_clearance_m', 0.52))
            return nearest is None or nearest >= clearance+self.clear_margin
        front = safety.get('front_m')
        stop = float(safety.get('front_stop_m', 0.65))
        return front is None or front >= stop+self.clear_margin

    def _fail(self, reason):
        self.failure = reason
        self.reason = reason
        self.state = self.IDLE

    def _start_replanning(self, now):
        self.state = self.REPLANNING
        self.reason = 'new A* route'
        self.replan_started = now
        self.replan_requested = True

    # ----------------------------------------------------------------- update
    def update(self, now, pose, safety, goal_active, path_ready=False):
        """Return ``None`` (let the goal follower drive) or ``(linear, angular)``."""
        safety = safety or {}
        if not goal_active:
            if self.state != self.IDLE or self.attempts:
                self.reset()
            return None
        if pose is None:
            return None
        if self.state in (self.IDLE, self.WAITING):
            return self._watch(now, pose, safety, path_ready)
        if self.state == self.BACKING:
            return self._move(now, pose, safety)
        if self.state == self.ROTATING:
            return self._rotate(now, pose, safety)
        return self._replan(now, path_ready, pose, safety)

    def _watch(self, now, pose, safety, path_ready=False):
        kind = self._blocked_kind(safety)
        # Safety reports an obstacle reason only while a motion command is
        # being stopped. The local planner holds zero during this wait, so keep
        # using the live scan distance to tell whether the obstacle is still
        # there.
        if kind is None and self.state == self.WAITING and self.kind == 'front':
            front = safety.get('front_m')
            stop = float(safety.get('front_stop_m', 0.65))
            if front is not None and front < stop + self.clear_margin:
                kind = 'front'
        elif kind is None and self.state == self.WAITING and self.kind == 'rotation':
            nearest = safety.get('nearest_m')
            clearance = float(safety.get('rotation_clearance_m', 0.52))
            if nearest is not None and nearest < clearance + self.clear_margin:
                kind = 'rotation'
        if self.state == self.IDLE:
            if kind:
                self.state = self.WAITING
                self.kind = kind
                self.blocked_since = self.last_blocked = now
                self.reason = 'obstacle ahead, waiting to see if it moves'
            return None
        if kind:
            self.kind = kind
            self.last_blocked = now
            if now-self.blocked_since >= self.wait_s:
                return self._begin(now, pose, safety)
            return (0.0, 0.0)
        if now-self.last_blocked >= self.debounce_s:
            if self.replan_started is not None and not path_ready:
                self.state = self.REPLANNING
                self.replan_started = now
                self.reason = 'obstacle cleared; waiting for an A* route'
                return (0.0, 0.0)
            self.state = self.IDLE            # it left: it was dynamic, resume
            self.reason = ''
        return None

    def _begin(self, now, pose, safety):
        if (self.attempt_origin is None
                or math.hypot(pose[0]-self.attempt_origin[0], pose[1]-self.attempt_origin[1]) > self.attempts_reset):
            self.attempts = 0
            self.attempt_origin = (pose[0], pose[1])
        self.attempts += 1
        front, rear = safety.get('front_m'), safety.get('rear_m')
        rear_stop = float(safety.get('rear_stop_m', 0.65))
        front_stop = float(safety.get('front_stop_m', 0.65))
        rear_free = rear is None or rear > rear_stop+0.10
        front_free = front is None or front > front_stop+0.10
        turn_ok = bool(safety.get('turn_allowed', False))
        self.phase_started, self.start_pose = now, (pose[0], pose[1])
        direction = 0
        if self.kind == 'front':
            direction = -1 if rear_free else 0
        else:
            bearing = safety.get('nearest_bearing_rad')
            ahead = bearing is None or abs(bearing) <= math.pi/2
            if ahead:
                direction = -1 if rear_free else (1 if front_free else 0)
            else:
                direction = 1 if front_free else (-1 if rear_free else 0)
        if direction:
            self.direction = direction
            self.state = self.BACKING
            self.reason = 'static obstacle: backing off' if direction < 0 else 'static obstacle: moving away'
            return (direction*self.move_speed, 0.0)
        if turn_ok:
            return self._begin_rotation(now, pose, safety)
        # Keep the original goal alive. If every direction is currently unsafe,
        # hold position and retry after another observation interval.
        self.state = self.WAITING
        self.blocked_since = self.last_blocked = now
        self.reason = 'no safe recovery direction; holding and retrying'
        return (0.0, 0.0)

    def _begin_rotation(self, now, pose, safety):
        left = safety.get('left_m')
        right = safety.get('right_m')
        left = 9.9 if left is None else left
        right = 9.9 if right is None else right
        self.rotate_sign = 1 if left >= right else -1
        self.rotated = 0.0
        self.last_yaw = pose[2]
        self.phase_started = now
        self.state = self.ROTATING
        self.reason = 'static obstacle: rotating towards free space'
        return (0.0, self.rotate_sign*self.rotate_rps)

    def _move(self, now, pose, safety):
        moved = math.hypot(pose[0]-self.start_pose[0], pose[1]-self.start_pose[1])
        elapsed = now-self.phase_started
        reason = str(safety.get('reason', ''))
        stopped = elapsed > 0.15 and reason.startswith('rear_obstacle' if self.direction < 0 else 'front_obstacle')
        if self._cleared(safety) or moved >= self.max_move or stopped or elapsed > self.phase_timeout:
            if not self._cleared(safety) and self.kind == 'front' and safety.get('turn_allowed', False):
                return self._begin_rotation(now, pose, safety)
            self._start_replanning(now)
            return (0.0, 0.0)
        return (self.direction*self.move_speed, 0.0)

    def _rotate(self, now, pose, safety):
        self.rotated += abs(_wrap(pose[2]-self.last_yaw))
        self.last_yaw = pose[2]
        reason = str(safety.get('reason', ''))
        stopped = now-self.phase_started > 0.15 and reason.startswith('rotation_obstacle')
        if (self._cleared(safety) or self.rotated >= self.max_rotate or stopped
                or now-self.phase_started > self.phase_timeout):
            self._start_replanning(now)
            return (0.0, 0.0)
        return (0.0, self.rotate_sign*self.rotate_rps)

    def _replan(self, now, path_ready, pose, safety):
        elapsed = now-self.replan_started
        if path_ready and elapsed >= self.replan_min:
            self.state = self.IDLE
            self.reason = ''
            return None
        if elapsed >= self.replan_timeout:
            # A* may temporarily have no route while the map catches up, or its
            # latest route may still meet an unmapped obstacle. Keep the goal
            # active and make another bounded, safety-checked recovery attempt.
            return self._begin(now, pose, safety)
        return (0.0, 0.0)
