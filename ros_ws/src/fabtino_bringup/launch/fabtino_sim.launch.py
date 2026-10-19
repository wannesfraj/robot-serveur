from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
def generate_launch_description():
    return LaunchDescription([
        Node(package='fabtino_webots_bridge',executable='fabtino-webots-bridge',name='webots_bridge',parameters=[{'host':'127.0.0.1','port':8766,'scan_period_s':0.10}]),
        # The Webots controller uses mecanum kinematics with K=L+W.  Since
        # this localization adapter currently reduces the four encoders to
        # left/right averages, its equivalent differential track is 2*K.
        Node(package='fabtino_localization',executable='fabtino-localization',name='localization',parameters=[{'track_width_m':1.043834,'wheel_radius_m':0.100,'lidar_angle_sign':-1.0,'use_imu_yaw_reference':True,'rotation_pending_wheel_rate_threshold':0.05,'rotation_confirm_gyro_rate_threshold':0.03,'rotation_active_gyro_rate_threshold':0.02,'rotation_in_place_linear_threshold':0.08,'rotation_settle_time_s':0.40}]),
        Node(package='fabtino_mapping',executable='fabtino-mapping',name='mapping',parameters=[{'resolution_m':0.10,'radius_m':20.0,'lidar_angle_sign':-1.0,'lidar_subsample':8,'scan_queue_size':256,'pose_history_size':512,'pose_match_tolerance_s':0.10,'publish_period_s':0.05,'map_publish_period_s':0.50}]),
        Node(package='fabtino_navigation',executable='fabtino-mission-manager',name='mission_manager',parameters=[{'state_file':str(Path.home()/'.local/share/fabtino/navigation.json'),'battery_required':True,'battery_auto_return_margin_pct':0.0}]),
        # Battery model (placeholder coefficients: calibrate on the real robot).
        # consumption_scale>1 drains faster in simulation; reserve_pct is the refusal threshold.
        Node(package='fabtino_navigation',executable='fabtino-battery',name='battery',parameters=[{'capacity_wh':480.0,'initial_soc_pct':100.0,'reserve_pct':15.0,'warning_pct':20.0,'safety_factor':1.25,'consumption_scale':1.0}]),
        Node(package='fabtino_navigation',executable='fabtino-global-planner',name='global_planner',parameters=[{'resolution_m':0.10,'radius_m':20.0,'robot_radius_m':0.52,'robot_width_m':0.70,'safety_margin_m':0.08,'allow_unknown':True,'unknown_cost':1.15,'battery_required':True,
            # [FIX] memoire de la forme des obstacles + cout doux pres des obstacles
            'lidar_angle_sign':-1.0,'obstacle_memory_range_m':2.0,'static_obstacle_ttl_s':60.0,
            'obstacle_clear_tolerance_m':0.25,'obstacle_cost_band_m':0.30,'obstacle_cost_weight':3.0}]),
        Node(package='fabtino_navigation',executable='fabtino-local-planner',name='local_planner',parameters=[{
            'path_max_angular_rps':0.70, 'path_heading_kd':0.35,
            'path_turn_timeout_s':20.0, 'path_turn_no_progress_s':5.0,
            'movement_no_progress_s':15.0,
            'goal_tolerance_m':0.05, 'yaw_tolerance_rad':0.05235987756,
            'final_approach_distance_m':0.40, 'final_linear_mps':0.08,
            'final_max_angular_rps':0.35, 'final_heading_kp':1.2, 'final_heading_kd':0.35,
            'stopped_linear_mps':0.01, 'stopped_angular_rps':0.03, 'settle_time_s':0.40,
            'yaw_hysteresis_rad':0.03490658504, 'alignment_no_progress_s':5.0,
            'alignment_timeout_s':30.0, 'pose_timeout_s':0.30,
            # Obstacle recovery: wait for a dynamic obstacle to pass, then move
            # away safely and keep retrying A* until a route becomes available.
            'recovery_wait_s':3.5, 'recovery_move_speed_mps':0.10, 'recovery_rotate_rps':0.45,
            'recovery_clear_margin_m':0.30, 'recovery_max_move_m':0.50,
            # [FIX] reculer jusqu'a une zone ou le robot peut pivoter, recul
            # progressif, limite de tentatives (0 = infini), evasion si pas de route.
            'recovery_turn_margin_m':0.10, 'recovery_max_escape_m':1.5,
            'recovery_max_attempts':6, 'recovery_no_route_s':5.0}]),
        Node(package='fabtino_safety',executable='fabtino-safety',name='safety_supervisor',parameters=[{'front_stop_distance_m':0.42,'rear_stop_distance_m':0.42,'braking_deceleration_mps2':0.8,'braking_reaction_time_s':0.12,
            # [FIX] convention d'angle Webots + virages doux autorises pres d'une chaise
            'lidar_angle_sign':-1.0,'arc_min_radius_m':0.6,'arc_side_clearance_m':0.45}]),
        Node(package='fabtino_viewer',executable='fabtino-viewer-gateway',name='viewer_gateway'),
    ])
