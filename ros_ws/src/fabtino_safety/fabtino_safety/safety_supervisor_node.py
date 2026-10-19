from __future__ import annotations
import math, json, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from fabtino_core.geometry import euler_from_quaternion
from fabtino_core.navigation.zones import motion_denied
class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor');self.declare_parameter('decision_timeout_s',0.30);self.declare_parameter('sensor_timeout_s',0.50);self.declare_parameter('front_stop_distance_m',0.65);self.declare_parameter('front_half_width_m',0.35)
        self.declare_parameter('rotation_clearance_m',0.52)
        self.declare_parameter('rear_stop_distance_m',0.65)
        self.declare_parameter('rear_half_width_m',0.35)
        self.declare_parameter('braking_deceleration_mps2',0.8)
        self.declare_parameter('braking_reaction_time_s',0.12)
        self.declare_parameter('teleop_timeout_s',0.30)
        # [FIX] Webots publie le scan avec +angle vers la DROITE du robot (meme
        # convention que mapping / global_planner : lidar_angle_sign=-1). Sans ce
        # signe, gauche/droite etaient inverses et le pivot de degagement partait
        # du cote le plus encombre.
        self.declare_parameter('lidar_angle_sign',-1.0)
        # [FIX] rotation_clearance_m concerne le PIVOT SUR PLACE (les coins balaient
        # un cercle). Avant, toute commande avec omega != 0 etait bloquee des qu'un
        # objet etait a moins de 0.52 m : le robot ne pouvait meme plus corriger son
        # cap en longeant une chaise -> blocage. Un virage en roulant (rayon >=
        # arc_min_radius_m) est maintenant autorise si rien n'est a moins de
        # arc_side_clearance_m; sinon le robot peut au moins avancer tout droit
        # (couloir avant verifie), sans braquer.
        self.declare_parameter('arc_min_radius_m',0.6)
        self.declare_parameter('arc_side_clearance_m',0.45)
        self.nav=None;self.teleop=None;self.last_scan=None;self.last_odom=None;self.last_nav_stamp=0.0;self.teleop_active=False;self.stop_latched=False
        self.navigation_enabled=True
        self.active_goal_id=None
        self.zones=[]
        self.config_revision=-1
        self.declare_parameter('navigation_clearance_m',.60)
        self.create_subscription(String,'/navigation/config',self.config_cb,10)
        self.pub=self.create_publisher(Twist,'/fabtino/cmd_vel',20);self.status=self.create_publisher(String,'/fabtino/safety_status',10)
        self.create_subscription(TwistStamped,'/navigation/cmd_vel',self.nav_cb,20);self.create_subscription(TwistStamped,'/teleop/cmd_vel',self.teleop_cb,20);self.create_subscription(LaserScan,'/fabtino/scan',self.scan_cb,10);self.create_subscription(Odometry,'/fabtino/odometry/filtered',self.odom_cb,10);self.create_subscription(String,'/safety/emergency_stop',self.emergency_cb,10);self.create_timer(0.02,self.loop)
        self.create_subscription(String,'/navigation/goal',self.goal_cb,10)
        self.create_subscription(String,'/navigation/local_status',self.local_status_cb,10)
    def nav_cb(self,m):self.nav=m;self.last_nav_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def config_cb(self,msg):
        payload=json.loads(msg.data);revision=int(payload.get('revision',0))
        if revision<self.config_revision:return
        self.config_revision=revision;self.zones=payload.get('zones',[])
    def teleop_cb(self,m):self.teleop=m;self.teleop_active=abs(m.twist.linear.x)>1e-6 or abs(m.twist.angular.z)>1e-6
    def scan_cb(self,m):self.last_scan=m
    def odom_cb(self,m):self.last_odom=m
    def emergency_cb(self,m):self.stop_latched=m.data.lower() not in ('','0','false','clear')
    def clear_motion(self):
        self.nav=None
        self.teleop=None
        self.teleop_active=False
        self.pub.publish(Twist())

    def stopping_distance(self, minimum_m, speed_mps):
        """Keep a small body-clearance floor and add distance for speed/reaction."""
        deceleration=max(0.05,float(self.get_parameter('braking_deceleration_mps2').value))
        reaction=max(0.0,float(self.get_parameter('braking_reaction_time_s').value))
        speed=abs(float(speed_mps))
        return float(minimum_m)+speed*reaction+speed*speed/(2.0*deceleration)

    def goal_cb(self,msg):
        payload=json.loads(msg.data)
        if 'navigation_config' in payload:
            config=String();config.data=json.dumps(dict(payload['navigation_config'],revision=payload.get('config_revision',0)));self.config_cb(config)
        self.navigation_enabled=payload.get('type')!='stop'
        self.active_goal_id=payload.get('goal_id') if self.navigation_enabled else None
        self.clear_motion()

    def local_status_cb(self,msg):
        payload=json.loads(msg.data)
        if (self.active_goal_id is not None and payload.get('goal_id')==self.active_goal_id
                and payload.get('state') in ('goal_reached','goal_failed')):
            self.navigation_enabled=False
            self.clear_motion()

    def loop(self):
        out=Twist();now=self.get_clock().now().nanoseconds*1e-9;reason='ok';turn_free=False;steering_limited=False
        # Free-space report for the obstacle-recovery logic (does not affect the stop decision).
        metrics=dict(front_m=None,front_bearing_rad=None,rear_m=None,rear_bearing_rad=None,
                     left_m=None,right_m=None,nearest_m=None,nearest_bearing_rad=None)
        # A released/lost browser key must not keep overriding autonomy forever.
        if self.teleop is not None:
            teleop_age=now-(self.teleop.header.stamp.sec+self.teleop.header.stamp.nanosec*1e-9)
            if teleop_age<0 or teleop_age>float(self.get_parameter('teleop_timeout_s').value):
                self.teleop=None
                self.teleop_active=False
        if self.stop_latched:reason='emergency_stop'
        elif self.last_scan is None or self.last_odom is None:reason='sensor_not_ready'
        elif now-(self.last_scan.header.stamp.sec+self.last_scan.header.stamp.nanosec*1e-9)>float(self.get_parameter('sensor_timeout_s').value):reason='sensor_timeout'
        elif now-(self.last_odom.header.stamp.sec+self.last_odom.header.stamp.nanosec*1e-9)>float(self.get_parameter('sensor_timeout_s').value):reason='localization_timeout'
        source='teleop' if self.teleop_active and self.teleop else ('navigation' if self.nav and self.navigation_enabled else 'idle')
        cmd=self.teleop.twist if source=='teleop' else (self.nav.twist if source=='navigation' else out)
        if reason=='ok' and source=='navigation':
            age=now-self.last_nav_stamp
            if age<0 or age>float(self.get_parameter('decision_timeout_s').value):reason=f'decision_age={age:.3f}s'
            elif self.zones:
                q=self.last_odom.pose.pose.orientation
                pose=(self.last_odom.pose.pose.position.x,self.last_odom.pose.pose.position.y,
                      euler_from_quaternion([q.x,q.y,q.z,q.w])[2])
                zone=motion_denied(pose,cmd.linear.x,cmd.angular.z,self.zones,
                    float(self.get_parameter('navigation_clearance_m').value))
                if zone:reason='denied_zone='+zone['name']
        if reason=='ok' and self.last_scan:
            measured_speed=(0.0 if self.last_odom is None else self.last_odom.twist.twist.linear.x)
            requested_speed=max(abs(measured_speed),abs(float(cmd.linear.x)))
            half=float(self.get_parameter('front_half_width_m').value)
            stopd=self.stopping_distance(self.get_parameter('front_stop_distance_m').value,requested_speed);best=None
            rear_half=float(self.get_parameter('rear_half_width_m').value)
            rear_stop=self.stopping_distance(self.get_parameter('rear_stop_distance_m').value,requested_speed)
            rear_obstacle=None
            rotation_clearance=float(self.get_parameter('rotation_clearance_m').value)
            rotation_obstacle=None
            min_range=float(getattr(self.last_scan,'range_min',0.0))
            max_range=float(getattr(self.last_scan,'range_max',math.inf))
            sign=-1.0 if float(self.get_parameter('lidar_angle_sign').value)<0 else 1.0
            for i,d in enumerate(self.last_scan.ranges):
                # Webots uses -1.0 for a LiDAR ray with no return.  It is not
                # an obstacle and must never trigger the forward safety stop.
                if not math.isfinite(d) or d <= 0.0 or d<min_range or d>max_range:continue
                a=self.last_scan.angle_min+i*self.last_scan.angle_increment
                x,y=d*math.cos(a),sign*d*math.sin(a)   # repere robot : +x avant, +y gauche
                if 0<x<1.5 and abs(y)<half and (metrics['front_m'] is None or d<metrics['front_m']):
                    metrics['front_m']=d;metrics['front_bearing_rad']=a
                if -1.5<x<0 and abs(y)<rear_half and (metrics['rear_m'] is None or d<metrics['rear_m']):
                    metrics['rear_m']=d;metrics['rear_bearing_rad']=a
                if d<1.2:
                    side='left_m' if y>0 else 'right_m'
                    metrics[side]=d if metrics[side] is None else min(metrics[side],d)
                if metrics['nearest_m'] is None or d<metrics['nearest_m']:metrics['nearest_m']=d;metrics['nearest_bearing_rad']=a
                # Check the robot-width corridor, not a narrow angular cone
                # that misses nearby chair/table legs beside the centreline.
                if 0<x<stopd and abs(y)<half:best=d if best is None else min(best,d)
                if -rear_stop<x<0 and abs(y)<rear_half:
                    rear_obstacle=d if rear_obstacle is None else min(rear_obstacle,d)
                if d<rotation_clearance:rotation_obstacle=d if rotation_obstacle is None else min(rotation_obstacle,d)
            turn_free=rotation_obstacle is None   # rien à moins de rotation_clearance_m : on peut pivoter
            if best is not None and cmd.linear.x>0:reason=f'front_obstacle={best:.2f}m'
            elif rear_obstacle is not None and cmd.linear.x<0:reason=f'rear_obstacle={rear_obstacle:.2f}m'
            elif rotation_obstacle is not None and abs(cmd.angular.z)>1e-6:
                v=float(cmd.linear.x);w=abs(float(cmd.angular.z))
                arc=v>0.03 and v/w>=float(self.get_parameter('arc_min_radius_m').value)
                if arc and rotation_obstacle>=float(self.get_parameter('arc_side_clearance_m').value):
                    pass                                        # virage doux en roulant : autorise
                elif v>0.03:
                    steering_limited=True                       # avance droite seulement
                else:
                    reason=f'rotation_obstacle={rotation_obstacle:.2f}m'
        if reason=='ok' and steering_limited:out.linear.x=cmd.linear.x
        elif reason=='ok':out.linear.x=cmd.linear.x;out.angular.z=cmd.angular.z
        elif turn_free and reason.startswith(('front_obstacle','rear_obstacle')):out.angular.z=cmd.angular.z  # avance bloquée, rotation permise pour contourner
        status_payload={
            'stop_active':reason!='ok','reason':reason,'command_source':source,
            'turn_allowed':turn_free,'steering_limited':steering_limited,'output_linear_mps':out.linear.x,
            'output_angular_rps':out.angular.z,
            'decision_age_s':(now-self.last_nav_stamp if self.last_nav_stamp else None),
            'front_stop_m':self.stopping_distance(
                self.get_parameter('front_stop_distance_m').value,
                max(abs(float(cmd.linear.x)),0.0 if self.last_odom is None
                    else abs(self.last_odom.twist.twist.linear.x))),
            'rear_stop_m':self.stopping_distance(
                self.get_parameter('rear_stop_distance_m').value,
                max(abs(float(cmd.linear.x)),0.0 if self.last_odom is None
                    else abs(self.last_odom.twist.twist.linear.x))),
            'rotation_clearance_m':float(self.get_parameter('rotation_clearance_m').value),
            **{k:(None if v is None else round(v,3)) for k,v in metrics.items()}}
        self.pub.publish(out)
        status=String();status.data=json.dumps(status_payload);self.status.publish(status)
def main():rclpy.init();n=SafetySupervisor();rclpy.spin(n)
if __name__=='__main__':main()
