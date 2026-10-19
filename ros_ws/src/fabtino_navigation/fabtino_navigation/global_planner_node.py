from __future__ import annotations
import json
import math
from dataclasses import replace
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from fabtino_core.navigation.planner import OccupancyGrid as CoreGrid, astar
from fabtino_core.geometry import euler_from_quaternion
from fabtino_core.localization.frames import lidar_return_to_world
from fabtino_navigation.goal_control import Goal
from fabtino_core.navigation.zones import point_denied,segment_denied,zone_mask
from fabtino_core.battery import BatteryConfig,evaluate_trip


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        for name, value in dict(resolution_m=0.10, radius_m=20.0, safety_margin_m=0.08,
                                robot_radius_m=0.52,
                                rotation_clearance_m=0.52,
                                robot_width_m=0.70,
                                allow_unknown=True, unknown_cost=1.15).items():
            self.declare_parameter(name, value)
        # battery_required=False keeps the old behaviour when no battery node runs.
        # After a safety stop the robot can sit inside the obstacle inflation band (about 0.43 m
        # for the configured footprint and margin), where A* has no free start cell. Inside this radius around the
        # robot the inflation (NOT the raw obstacle cells, NOT denied zones) is relaxed so A* can plan the way out.
        self.declare_parameter('start_escape_radius_m', 0.5)
        self.declare_parameter('start_escape_hard_margin_m', 0.20)
        self.declare_parameter('battery_required', False)
        self.declare_parameter('battery_max_age_s', 5.0)
        self.declare_parameter('battery_wait_timeout_s', 5.0)
        # Safety detections supplement the occupancy map during recovery. This
        # catches close obstacles before the next mapping publication arrives.
        self.declare_parameter('lidar_angle_sign', -1.0)
        self.declare_parameter('transient_obstacle_ttl_s', 6.0)
        self.declare_parameter('transient_obstacle_max_distance_m', 1.2)
        # [FIX] Memoire de la FORME de l'obstacle. Avant, A* ne recevait qu'UN point
        # (le retour LiDAR le plus proche) pendant 6 s : il contournait ce point et
        # renvoyait le robot dans le bord de la table -> boucle infinie.
        # Maintenant, pendant un arret/une reprise, tous les points du scan proches
        # sont ajoutes; un obstacle confirme STATIQUE est garde longtemps, et il est
        # efface des que le LiDAR voit a travers (l'objet a ete deplace).
        self.declare_parameter('obstacle_memory_range_m', 2.0)
        self.declare_parameter('static_obstacle_ttl_s', 60.0)
        self.declare_parameter('obstacle_memory_max_cells', 6000)
        self.declare_parameter('obstacle_clear_tolerance_m', 0.25)
        self.last_scan = None
        self.local_static = False
        # [FIX] cout doux pres des obstacles : A* garde de la distance quand il y a
        # la place, au lieu de raser la limite du gonflage (zone ou le robot ne peut
        # plus pivoter).
        self.declare_parameter('obstacle_cost_band_m', 0.30)
        self.declare_parameter('obstacle_cost_weight', 3.0)
        self.battery = None
        self.battery_stamp = None
        self.battery_refused = None
        self.battery_report = None
        self.battery_waiting_goal = None
        self.grid = None
        self.pose = None
        self.pose_yaw = None
        self.goal = None
        self.local_recovering = False
        self.goal_started_at = None
        self.return_to_base = False
        self.path = []
        self.transient_obstacles = {}
        self.last_no_path_goal = None
        self.zones=[]
        self.base=None
        self.config_revision=-1
        self.last_navigation_map_ns=None
        self.pub = self.create_publisher(NavPath, '/navigation/global_path', 5)
        # Keep standard Path for visualization; tagged routes prevent stale motion.
        self.route_pub = self.create_publisher(String, '/navigation/route', 5)
        self.status = self.create_publisher(String, '/navigation/global_status', 10)
        self.create_subscription(OccupancyGrid, '/fabtino/map', self.map_cb, 3)
        self.create_subscription(OccupancyGrid,'/fabtino/navigation_map',lambda msg:self.map_cb(msg,True),3)
        self.create_subscription(String,'/navigation/config',self.config_cb,10)
        self.create_subscription(String,'/fabtino/battery',self.battery_cb,10)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.pose_cb, 10)
        self.create_subscription(String, '/navigation/goal', self.goal_cb, 10)
        self.create_subscription(String, '/navigation/local_status', self.local_cb, 10)
        self.create_subscription(String, '/fabtino/safety_status', self.safety_cb, 20)
        self.create_subscription(LaserScan, '/fabtino/scan', self.scan_cb, 5)
        self.create_timer(0.1, self.plan)

    def map_cb(self, msg, navigation=False):
        now=self.get_clock().now().nanoseconds
        if navigation:self.last_navigation_map_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        elif self.last_navigation_map_ns is not None and now-self.last_navigation_map_ns<600_000_000:return
        resolution = float(msg.info.resolution)
        radius = -float(msg.info.origin.position.x)
        self.grid = CoreGrid(resolution, radius)
        self.grid.load(resolution, radius, msg.data)

    def config_cb(self,msg):
        payload=json.loads(msg.data);revision=int(payload.get('revision',0))
        if revision<self.config_revision:return
        zones=payload.get('zones',[]);base=payload.get('base')
        changed=zones!=self.zones or base!=self.base
        self.config_revision=revision;self.zones=zones;self.base=base
        if changed:
            self.path=[];self.publish_path()

    def pose_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        self.pose_yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]

    def safety_cb(self, msg):
        if self.goal is None or self.pose is None or self.pose_yaw is None:
            return
        try:payload=json.loads(msg.data)
        except ValueError:return
        reason=str(payload.get('reason',''))
        obstacle_stop=bool(payload.get('stop_active')) and reason.startswith(
            ('front_obstacle','rotation_obstacle'))
        front=payload.get('front_m')
        bearing=payload.get('front_bearing_rad')
        if (obstacle_stop or self.local_recovering) and front is not None and bearing is not None:
            try:distance=float(front);angle=float(bearing)
            except (TypeError,ValueError):distance=angle=math.nan
            max_distance=float(self.get_parameter('transient_obstacle_max_distance_m').value)
            # Keep the active obstruction in A* for the short recovery window.
            # Once it moves away its cell expires automatically.
            if math.isfinite(distance) and math.isfinite(angle) and 0.0<distance<=max_distance:
                self.remember_obstacle(distance,angle)
        elif obstacle_stop or self.local_recovering:
            distance=payload.get('nearest_m');angle=payload.get('nearest_bearing_rad')
            try:distance=float(distance);angle=float(angle)
            except (TypeError,ValueError):return
            if math.isfinite(distance) and math.isfinite(angle) and distance>0.0:
                self.remember_obstacle(distance,angle)

    def scan_cb(self, msg):
        self.last_scan = msg
        if self.pose is None or self.pose_yaw is None:
            return
        self.clear_seen_through(msg)
        if self.goal is None or not (self.local_recovering or self.local_static):
            return
        # Robot arrete devant un obstacle : memoriser TOUT ce que le LiDAR voit pres de lui.
        ttl = (float(self.get_parameter('static_obstacle_ttl_s').value) if self.local_static
               else float(self.get_parameter('transient_obstacle_ttl_s').value))
        reach = float(self.get_parameter('obstacle_memory_range_m').value)
        rmin = float(getattr(msg, 'range_min', 0.0))
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d) or d <= 0.0 or d < rmin or d > reach:
                continue
            self.remember_obstacle(d, msg.angle_min+i*msg.angle_increment, ttl)

    def clear_seen_through(self, msg):
        """Forget remembered cells the LiDAR now sees through (obstacle moved away)."""
        if not self.transient_obstacles or not msg.ranges or not msg.angle_increment:
            return
        sign = -1.0 if float(self.get_parameter('lidar_angle_sign').value) < 0 else 1.0
        tol = float(self.get_parameter('obstacle_clear_tolerance_m').value)
        rmin = float(getattr(msg, 'range_min', 0.0))
        rmax = min(float(getattr(msg, 'range_max', 12.0)), 4.0)
        n = len(msg.ranges)
        c, s_ = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        gone = []
        for cell, (wx, wy, _expiry) in self.transient_obstacles.items():
            dx, dy = wx-self.pose[0], wy-self.pose[1]
            xr, yr = c*dx+s_*dy, -s_*dx+c*dy          # robot frame
            dist = math.hypot(xr, yr)
            if dist < max(rmin, 0.3) or dist > rmax:
                continue
            raw = sign*math.atan2(yr, xr)              # angle in the scan's own convention
            centre = int(round((raw-msg.angle_min)/msg.angle_increment))
            free = True
            for k in (-1, 0, 1):
                d = msg.ranges[(centre+k) % n]
                hit = math.isfinite(d) and d > 0.0 and d >= rmin
                if hit and d <= dist+tol:
                    free = False
                    break
            if free:
                gone.append(cell)
        for cell in gone:
            self.transient_obstacles.pop(cell, None)

    def remember_obstacle(self, distance, angle, ttl=None):
        world=lidar_return_to_world(
            (self.pose[0],self.pose[1],self.pose_yaw),distance,angle,
            float(self.get_parameter('lidar_angle_sign').value))
        if self.grid is not None:
            cell=self.grid.cell(*world)
        else:
            resolution=float(self.get_parameter('resolution_m').value)
            radius=float(self.get_parameter('radius_m').value)
            cell=(int(math.floor((world[0]+radius)/resolution)),
                  int(math.floor((world[1]+radius)/resolution)))
        if cell is None:return
        if ttl is None:
            ttl=(float(self.get_parameter('static_obstacle_ttl_s').value) if self.local_static
                 else float(self.get_parameter('transient_obstacle_ttl_s').value))
        now=self.get_clock().now().nanoseconds*1e-9
        expiry=now+max(0.5,ttl)
        previous=self.transient_obstacles.get(cell)
        if previous is not None and previous[2]>expiry:expiry=previous[2]   # never shorten a static memory
        if previous is None and len(self.transient_obstacles)>=int(self.get_parameter('obstacle_memory_max_cells').value):
            oldest=min(self.transient_obstacles,key=lambda k:self.transient_obstacles[k][2])
            self.transient_obstacles.pop(oldest,None)
        self.transient_obstacles[cell]=(world[0],world[1],expiry)

    def battery_cb(self, msg):
        try:self.battery = json.loads(msg.data)
        except ValueError:return
        self.battery_stamp = self.get_clock().now().nanoseconds*1e-9

    def battery_check(self, points):
        """Return None when the trajectory is acceptable, else (state, reason)."""
        required = bool(self.get_parameter('battery_required').value)
        age = None if self.battery_stamp is None else self.get_clock().now().nanoseconds*1e-9-self.battery_stamp
        if self.battery is None or age is None or age > float(self.get_parameter('battery_max_age_s').value):
            if required:
                return 'battery_insufficient', 'Battery level unknown (no recent /fabtino/battery message)'
            return None
        try:
            config = BatteryConfig.from_dict(self.battery['config'])
            config = replace(config, reserve_pct=0.0 if self.return_to_base else 15.0,
                             warning_pct=20.0)
            # A return-to-dock route may use the protected reserve; it only needs
            # enough energy to reach the charger, with the model's safety margin.
            if self.return_to_base:
                config = replace(config, reserve_pct=0.0)
            report = evaluate_trip(points, float(self.battery['soc_pct']), config)
        except (KeyError, TypeError, ValueError) as exc:
            return ('battery_insufficient', f'Invalid battery data: {exc}') if required else None
        self.battery_report = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in report.items() if k != 'reason'}
        return None if report['accepted'] else ('battery_insufficient', report['reason'])

    def battery_precheck(self):
        """Reject a provably infeasible trip before running A* or publishing a route."""
        if not bool(self.get_parameter('battery_required').value):
            return None
        now=self.get_clock().now().nanoseconds*1e-9
        age=None if self.battery_stamp is None else now-self.battery_stamp
        max_age=float(self.get_parameter('battery_max_age_s').value)
        if self.battery is None or age is None or age>max_age:
            waited=0.0 if self.goal_started_at is None else now-self.goal_started_at
            if waited<float(self.get_parameter('battery_wait_timeout_s').value):
                return 'waiting'
            return ('battery_insufficient','Battery level unavailable; refusing to start without a fresh reading')
        try:
            config=BatteryConfig.from_dict(self.battery['config'])
            config=replace(config,reserve_pct=0.0 if self.return_to_base else 15.0,
                           warning_pct=20.0)
            if not self.return_to_base:
                # A straight-line estimate with turning cost removed is a lower
                # bound. If even this fails, there is no need to search the map.
                config=replace(config,wh_per_rad=0.0)
            points=[tuple(self.pose),(self.goal.x,self.goal.y)]
            if not self.return_to_base and self.base is not None:
                points.append((float(self.base['x']),float(self.base['y'])))
            report=evaluate_trip(points,float(self.battery['soc_pct']),config)
        except (KeyError,TypeError,ValueError) as exc:
            return 'battery_insufficient',f'Invalid battery data: {exc}'
        if report['accepted']:
            return None
        self.battery_report={k:(round(v,3) if isinstance(v,float) else v)
                             for k,v in report.items() if k!='reason'}
        self.battery_report['minimum_estimate']=True
        return ('battery_insufficient',
                'Battery too low even for the minimum route estimate: '+report['reason'])

    def goal_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if 'navigation_config' in payload:
                config=String();config.data=json.dumps(dict(payload['navigation_config'],revision=payload.get('config_revision',0)));self.config_cb(config)
            goal = None if payload.get('type') == 'stop' else Goal.from_payload(payload)
            self.goal = goal
            self.local_recovering = False
            self.goal_started_at = self.get_clock().now().nanoseconds*1e-9 if goal is not None else None
            self.return_to_base = bool(payload.get('return_to_base',False))
            self.path = []
            # [FIX] garder les obstacles statiques confirmes pour le trajet suivant
            # (A -> B -> base) : seuls les points courts (dynamiques) sont oublies.
            now=self.get_clock().now().nanoseconds*1e-9
            short=float(self.get_parameter('transient_obstacle_ttl_s').value)+1e-6
            self.transient_obstacles={c:o for c,o in self.transient_obstacles.items()
                                      if o[2]-now>short}
            self.last_no_path_goal = None
            self.battery_refused = self.battery_report = None
            self.battery_waiting_goal = None
            self.publish_path()
            self.emit('stopped' if goal is None else 'planning')
        except (ValueError, KeyError, TypeError) as exc:
            self.emit('invalid_goal', str(exc))

    def local_cb(self, msg):
        payload = json.loads(msg.data)
        self.local_recovering = bool(
            self.goal is not None and payload.get('goal_id') == self.goal.goal_id
            and payload.get('state') == 'recovering')
        self.local_static = bool(self.local_recovering and (payload.get('recovery') or {}).get('static'))
        if (self.goal is not None and payload.get('goal_id') == self.goal.goal_id
                and payload.get('state') in ('goal_reached', 'goal_failed')):
            self.emit(payload['state'], payload.get('reason', ''))
            self.goal = None
            self.path = []
            self.publish_path()

    def emit(self, state, reason=''):
        msg = String()
        msg.data = json.dumps({'state': state, 'reason': reason, 'battery': self.battery_report,
                               'goal_id': self.goal.goal_id if self.goal else None,
                               'goal': [self.goal.x, self.goal.y] if self.goal else None,
                               'path_cells': len(self.path)})
        self.status.publish(msg)

    def fail(self, state, reason=''):
        self.path = []
        self.publish_path()
        self.emit(state, reason)

    def plan(self):
        if self.goal is None:
            return
        if self.battery_refused == self.goal.goal_id:
            return          # refused for this goal; wait for the mission manager to cancel it
        if self.pose is not None:
            precheck=self.battery_precheck()
            if precheck=='waiting':
                if self.battery_waiting_goal!=self.goal.goal_id:
                    self.battery_waiting_goal=self.goal.goal_id
                    self.emit('waiting_for_battery','Waiting for a fresh battery reading')
                return
            self.battery_waiting_goal=None
            if precheck is not None:
                self.battery_refused=self.goal.goal_id
                self.fail(*precheck)
                return
        if self.grid is None or self.pose is None:
            self.fail('waiting_for_map_or_pose')
            return
        start = self.grid.cell(*self.pose)
        goal = self.grid.cell(self.goal.x, self.goal.y)
        if goal is None:
            self.fail('goal_outside_map')
            return
        if start is None:
            self.fail('outside_map_recovery', 'Robot is outside the navigation map')
            return
        zone_clearance = (float(self.get_parameter('robot_radius_m').value)
                          + float(self.get_parameter('safety_margin_m').value))
        # Translation through a corridor needs the robot's half-width plus margin.
        # Turning clearance is enforced separately by the safety supervisor.
        passage_clearance = max(
            float(self.get_parameter('robot_width_m').value)/2.0
            + float(self.get_parameter('safety_margin_m').value),
            float(self.get_parameter('rotation_clearance_m').value))
        if point_denied(self.pose[0],self.pose[1],self.zones,zone_clearance):
            self.fail('robot_in_denied_zone','Move the robot outside the denied zone before navigating');return
        if point_denied(self.goal.x,self.goal.y,self.zones,zone_clearance):
            self.fail('goal_in_denied_zone','Choose a target outside the denied zone and robot clearance');return
        now_seconds=self.get_clock().now().nanoseconds*1e-9
        self.grid.restore_static()
        expired=[cell for cell,obstacle in self.transient_obstacles.items() if obstacle[2]<=now_seconds]
        for cell in expired:self.transient_obstacles.pop(cell,None)
        for x,y,_expiry in self.transient_obstacles.values():self.grid.set_dynamic_occupied(x,y)
        blocked = self.grid.inflated_obstacles(passage_clearance)|zone_mask(self.grid,self.zones,zone_clearance)
        if blocked[start[1],start[0]]:blocked=self.relax_start(blocked,start,zone_clearance)
        extra_cost=None
        band=float(self.get_parameter('obstacle_cost_band_m').value)
        weight=float(self.get_parameter('obstacle_cost_weight').value)
        if band>0 and weight>0:
            near=self.grid.inflated_obstacles(passage_clearance+band*0.5)
            far=self.grid.inflated_obstacles(passage_clearance+band)
            extra_cost=far.astype(float)*(weight*0.4)+near.astype(float)*(weight*0.6)
        def edge_allowed(a,b):
            if not self.zones or (a!=start and b!=goal):return True
            pa=self.pose if a==start else self.grid.point(a)
            pb=(self.goal.x,self.goal.y) if b==goal else self.grid.point(b)
            return not segment_denied(pa,pb,self.zones,zone_clearance)
        path = astar(self.grid, start, goal, blocked,
                     bool(self.get_parameter('allow_unknown').value),
                     float(self.get_parameter('unknown_cost').value),edge_allowed,extra_cost)
        if path:
            # Estimate the full route, including the return to base when one is
            # configured. This is refreshed as the robot moves and the battery drains.
            points=[self.grid.point(cell) for cell in path]
            points[0]=tuple(self.pose);points[-1]=(self.goal.x,self.goal.y)
            battery_age=(None if self.battery_stamp is None else
                         self.get_clock().now().nanoseconds*1e-9-self.battery_stamp)
            battery_fresh=(self.battery is not None and battery_age is not None and
                           battery_age<=float(self.get_parameter('battery_max_age_s').value))
            reserve_return=(self.base is not None and not self.return_to_base and
                            (bool(self.get_parameter('battery_required').value) or battery_fresh))
            if reserve_return:
                base_point=(float(self.base['x']),float(self.base['y']))
                base_cell=self.grid.cell(*base_point)
                return_path=[]
                if base_cell is not None:
                    def return_edge_allowed(a,b):
                        pa=(self.goal.x,self.goal.y) if a==goal else self.grid.point(a)
                        pb=base_point if b==base_cell else self.grid.point(b)
                        return not segment_denied(pa,pb,self.zones,zone_clearance)
                    return_path=astar(self.grid,goal,base_cell,blocked,
                                      bool(self.get_parameter('allow_unknown').value),
                                      float(self.get_parameter('unknown_cost').value),return_edge_allowed)
                if not return_path:
                    self.battery_refused=self.goal.goal_id
                    self.battery_report={'accepted':False,'return_path_available':False}
                    self.fail('battery_insufficient','Cannot reserve a safe route from the destination back to the configured base')
                    return
                return_points=[self.grid.point(cell) for cell in return_path]
                return_points[0]=(self.goal.x,self.goal.y);return_points[-1]=base_point
                if len(return_points)>1:points.extend(return_points[1:])
                else:points.append(base_point)
            verdict=self.battery_check(points)
            if verdict is not None:
                self.battery_refused=self.goal.goal_id
                self.fail(*verdict)
                return
            self.path = path
            self.last_no_path_goal = None
            self.publish_path()
            if self.path:self.emit('path_ready')
            else:self.emit('pathfinding_failed','The route intersects a denied zone')
        else:
            # Keep the original goal active while the map changes or obstacle
            # recovery moves the robot to a better planning position.
            self.path=[]
            self.publish_path()
            if self.last_no_path_goal!=self.goal.goal_id:
                self.emit('waiting_for_route','No A* route is available yet; keeping the goal active and retrying')
                self.last_no_path_goal=self.goal.goal_id

    def relax_start(self,blocked,start,zone_clearance):
        """Free the inflation-only cells within ``start_escape_radius_m`` of the robot."""
        radius=float(self.get_parameter('start_escape_radius_m').value)
        if radius<=0:return blocked
        hard=self.grid.inflated_obstacles(float(self.get_parameter('start_escape_hard_margin_m').value))
        if hard[start[1],start[0]]:return blocked           # really touching an obstacle: do not guess
        if self.zones and zone_mask(self.grid,self.zones,zone_clearance)[start[1],start[0]]:return blocked
        zones=zone_mask(self.grid,self.zones,zone_clearance) if self.zones else None
        cells=int(math.ceil(radius/self.grid.resolution));relaxed=blocked.copy()
        for gy in range(max(0,start[1]-cells),min(self.grid.size,start[1]+cells+1)):
            for gx in range(max(0,start[0]-cells),min(self.grid.size,start[0]+cells+1)):
                if math.hypot(gx-start[0],gy-start[1])*self.grid.resolution>radius or hard[gy,gx]:continue
                if zones is not None and zones[gy,gx]:continue
                relaxed[gy,gx]=False
        return relaxed

    def publish_path(self):
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        points = [self.grid.point(cell) for cell in self.path]
        if points and self.pose:points[0]=tuple(self.pose)
        if points and self.goal:
            # A* uses cell centres; the last target must be the requested position.
            points[-1] = (self.goal.x, self.goal.y)
        zone_clearance=float(self.get_parameter('robot_radius_m').value)+float(self.get_parameter('safety_margin_m').value)
        if any(segment_denied(a,b,self.zones,zone_clearance) for a,b in zip(points,points[1:])):
            self.path=[];points=[]
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        if msg.poses and self.goal.yaw_rad is not None:
            msg.poses[-1].pose.orientation.z = math.sin(self.goal.yaw_rad/2)
            msg.poses[-1].pose.orientation.w = math.cos(self.goal.yaw_rad/2)
        self.pub.publish(msg)
        route = String()
        route.data = json.dumps({'goal_id': self.goal.goal_id if self.goal else None,
                                 'points': points})
        self.route_pub.publish(route)


def main():
    rclpy.init()
    node = GlobalPlannerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
