from __future__ import annotations
import json
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from fabtino_navigation.goal_control import Goal, GoalFollower, wrap_angle
from fabtino_navigation.obstacle_recovery import ObstacleRecovery
from fabtino_core.navigation.zones import segment_denied


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        defaults = dict(lookahead_m=0.45, max_linear_mps=0.25, max_angular_rps=1.0,
                        heading_slowdown_rad=1.0, goal_tolerance_m=0.05,
                        yaw_tolerance_rad=math.radians(3), final_max_angular_rps=0.35,
                        settle_time_s=0.4, yaw_hysteresis_rad=math.radians(2),
                        alignment_no_progress_s=5.0, alignment_timeout_s=30.0,
                        final_approach_distance_m=0.4, final_linear_mps=0.08,
                        final_heading_kp=1.2, final_heading_kd=0.35,
                        stopped_linear_mps=0.01, stopped_angular_rps=0.03,
                        path_max_angular_rps=0.7, path_heading_kd=0.35,
                        path_turn_timeout_s=20.0, path_turn_no_progress_s=5.0,
                        movement_no_progress_s=15.0)
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.controller = GoalFollower(**{name: float(self.get_parameter(name).value) for name in defaults})
        self.pose = None
        self.velocity = (0.0, 0.0)
        self.pose_stamp = None
        self.declare_parameter('pose_timeout_s',0.30)
        self.declare_parameter('blocked_wait_s',60.0)
        self.safety_blocked_since=None;self.last_control_seconds=None
        # Obstacle recovery: wait, then (if still there) back off / rotate and let A* replan.
        recovery_defaults=dict(recovery_wait_s=3.5,recovery_move_speed_mps=0.10,recovery_rotate_rps=0.45,
                               recovery_clear_margin_m=0.30,recovery_max_move_m=0.50,
                               recovery_replan_timeout_s=10.0,
                               # [FIX] nouvelle logique "se placer dans une zone libre"
                               recovery_turn_margin_m=0.10,     # marge au-dela de rotation_clearance pour pouvoir pivoter
                               recovery_max_escape_m=1.5,       # recul max apres plusieurs echecs au meme endroit
                               recovery_max_attempts=6,         # 0 = reessayer indefiniment
                               recovery_no_route_s=5.0)         # pas de route A* apres un obstacle -> on se deplace
        for name,default in recovery_defaults.items():self.declare_parameter(name,default)
        self.declare_parameter('recovery_enabled',True)
        self.safety_status={}
        self.no_route_since=None
        self.recovery=ObstacleRecovery(
            wait_s=float(self.get_parameter('recovery_wait_s').value),
            move_speed_mps=float(self.get_parameter('recovery_move_speed_mps').value),
            rotate_rps=float(self.get_parameter('recovery_rotate_rps').value),
            clear_margin_m=float(self.get_parameter('recovery_clear_margin_m').value),
            max_move_m=float(self.get_parameter('recovery_max_move_m').value),
            replan_timeout_s=float(self.get_parameter('recovery_replan_timeout_s').value),
            turn_margin_m=float(self.get_parameter('recovery_turn_margin_m').value),
            max_escape_m=float(self.get_parameter('recovery_max_escape_m').value),
            max_attempts=int(self.get_parameter('recovery_max_attempts').value))
        self.create_subscription(String,'/fabtino/safety_status',self.safety_cb,20)
        self.last_status = None
        self.last_status_time = -math.inf
        self.zones=[]
        self.config_revision=-1
        self.declare_parameter('navigation_clearance_m',.60)
        self.create_subscription(String,'/navigation/config',self.config_cb,10)
        self.pub = self.create_publisher(TwistStamped, '/navigation/cmd_vel', 20)
        self.status = self.create_publisher(String, '/navigation/local_status', 10)
        self.create_subscription(String, '/navigation/goal', self.goal_cb, 10)
        self.create_subscription(String, '/navigation/route', self.path_cb, 5)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.pose_cb, 10)
        self.create_timer(0.05, self.control)

    def goal_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if 'navigation_config' in payload:
                config=String();config.data=json.dumps(dict(payload['navigation_config'],revision=payload.get('config_revision',0)));self.config_cb(config)
            self.recovery.reset()
            if payload.get('type') == 'stop':
                self.controller.stop()
            else:
                self.controller.set_goal(Goal.from_payload(payload))
            self.control()
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f'Invalid navigation goal: {exc}')

    def config_cb(self,msg):
        payload=json.loads(msg.data);revision=int(payload.get('revision',0))
        if revision<self.config_revision:return
        self.config_revision=revision;self.zones=payload.get('zones',[])

    def path_cb(self, msg):
        payload = json.loads(msg.data)
        self.controller.set_path(payload['goal_id'], payload['points'])

    def safety_cb(self, msg):
        try:payload=json.loads(msg.data)
        except ValueError:return
        self.safety_status=payload
        reason=str(payload.get('reason',''))
        blocked=bool(payload.get('stop_active')) and reason.startswith(('front_obstacle','rear_obstacle','rotation_obstacle'))
        if blocked and self.safety_blocked_since is None:self.safety_blocked_since=self.get_clock().now().nanoseconds*1e-9
        elif not blocked:self.safety_blocked_since=None

    def hold_timers(self, seconds):
        # Gèle les minuteurs « pas de progrès » pendant que la sécurité retient le robot.
        dt=0.0 if self.last_control_seconds is None else max(0.0,min(0.5,seconds-self.last_control_seconds))
        self.last_control_seconds=seconds
        since=self.safety_blocked_since;c=self.controller
        if self.recovery.active and c.goal is not None:
            for name in ('progress_time','turn_started','turn_last_progress','alignment_started','last_alignment_progress'):
                value=getattr(c,name,None)
                if value is not None:setattr(c,name,value+dt)
            return
        if since is None or c.goal is None or seconds-since>=float(self.get_parameter('blocked_wait_s').value):return
        for name in ('progress_time','turn_started','turn_last_progress','alignment_started','last_alignment_progress'):
            value=getattr(c,name,None)
            if value is not None:setattr(c,name,value+dt)

    def pose_cb(self, msg):
        from fabtino_core.geometry import euler_from_quaternion
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
        self.velocity = (msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        self.pose_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def recovery_or_follow(self, seconds):
        """Run the goal follower, unless an obstacle recovery maneuver owns the motors."""
        c = self.controller
        if not bool(self.get_parameter('recovery_enabled').value):
            return c.command(self.pose, seconds, velocity=self.velocity)
        # Only recover while driving along a route; final alignment/settling is left alone.
        driving = c.goal is not None and c.state in ('following', 'approaching', 'turning_to_path')
        self.recovery.goal_distance = (None if c.goal is None or self.pose is None
                                       else math.hypot(c.goal.x-self.pose[0], c.goal.y-self.pose[1]))
        override = self.recovery.update(seconds, self.pose, self.safety_status,
                                        driving or self.recovery.active, path_ready=bool(c.path))
        if self.recovery.failure:
            reason = self.recovery.failure
            self.recovery.reset()
            return c.fail(reason)
        if self.recovery.take_replan_request():
            # Drop the stale route and restart the no-progress timers; the next A* route
            # (computed from the new position) re-arms the follower.
            c.path = []
            c.reset_alignment()
            c.state = 'waiting_for_path'
            c.settled_since = None
        if override is not None:
            return override
        # [FIX] Apres un obstacle, si A* ne trouve aucune route depuis la position
        # actuelle (robot coince dans la zone gonflee de l'obstacle), on ne reste
        # pas immobile pour toujours : on se deplace vers une autre zone et on
        # redemande un chemin.
        waiting = (c.goal is not None and c.state == 'waiting_for_path' and not c.path
                   and not self.recovery.active and self.recovery.encountered_obstacle)
        if not waiting:
            self.no_route_since = None
        else:
            if self.no_route_since is None:
                self.no_route_since = seconds
            elif seconds-self.no_route_since >= float(self.get_parameter('recovery_no_route_s').value):
                self.no_route_since = None
                escape = self.recovery.force_escape(seconds, self.pose, self.safety_status)
                if self.recovery.failure:
                    reason = self.recovery.failure
                    self.recovery.reset()
                    return c.fail(reason)
                if escape is not None:
                    return escape
        return c.command(self.pose, seconds, velocity=self.velocity)

    def control(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds*1e-9
        pose_age = None if self.pose_stamp is None else seconds-self.pose_stamp
        pose_fresh = pose_age is not None and 0 <= pose_age <= float(self.get_parameter('pose_timeout_s').value)
        if not pose_fresh and self.controller.goal is not None and self.controller.state not in ('goal_reached','goal_failed'):
            linear, angular = 0.0, 0.0
            self.controller.state = 'waiting_for_pose'
            self.controller.settled_since = None
        else:
            lookahead=self.controller.lookahead
            self.hold_timers(seconds)
            try:
                if self.pose is not None and self.zones and self.controller.path:
                    while self.controller.lookahead>.025 and segment_denied(self.pose[:2],self.controller.arc_target(self.pose),
                            self.zones,float(self.get_parameter('navigation_clearance_m').value)):
                        self.controller.lookahead*=.5
                linear, angular = self.recovery_or_follow(seconds)
            finally:self.controller.lookahead=lookahead
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = linear, angular
        self.pub.publish(msg)
        goal_id = self.controller.goal.goal_id if self.controller.goal else None
        state_name = 'recovering' if self.recovery.active or self.recovery.state==ObstacleRecovery.WAITING else self.controller.state
        signature = (state_name, goal_id, self.recovery.state, self.recovery.attempts)
        if signature != self.last_status or seconds-self.last_status_time >= 0.5:
            status = String()
            goal = self.controller.goal
            distance = None if goal is None or self.pose is None else math.hypot(goal.x-self.pose[0], goal.y-self.pose[1])
            yaw_error = None if goal is None or goal.yaw_rad is None or self.pose is None else math.degrees(wrap_angle(goal.yaw_rad-self.pose[2]))
            status.data = json.dumps({'state': signature[0], 'goal_id': goal_id,
                                     'reason': self.recovery.reason if state_name=='recovering' else self.controller.reason,
                                     'recovery': self.recovery.snapshot(),
                                     'controller_version': 'arrival-v4',
                                     'distance_m': distance, 'yaw_error_deg': yaw_error,
                                     'measured_linear_mps':self.velocity[0],
                                     'measured_angular_rps':self.velocity[1], 'pose_age_s':pose_age,
                                     'position_reached': self.controller.position_reached,
                                     'command_linear_mps': linear, 'command_angular_rps': angular})
            self.status.publish(status)
            self.last_status = signature
            self.last_status_time = seconds


def main():
    rclpy.init()
    node = LocalPlannerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
