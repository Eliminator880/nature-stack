import os
from datetime import datetime
import launch
import launch_ros.actions
import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch.conditions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handler import EventHandler
from launch.events.process import ProcessExited
from launch.events import Shutdown as LaunchShutdown
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
import random
import math
def generate_launch_description():  

    vehicle_speed = LaunchConfiguration('vehicle_speed')
    vehicle_speed_arg = DeclareLaunchArgument(
        'vehicle_speed',
        default_value='5.0',
        description='Vehicle speed (m/s)'
    )
    
    bag_name = LaunchConfiguration('bag_name')
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='mavs_traverse_rosbag',
        description='bag name for the recording'
    )
    
    ystart = random.uniform(-0.5, 0.5)
    headstart = math.radians(90.0+random.uniform(-5.0,5.0))   
    global_scene_seed = 20 #random.randint(19, 250)
    use_sim_time = True
    nature_package_dir = get_package_share_directory('nature')
    urdf = os.path.join(nature_package_dir, 'config', 'example_bot.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    #waypoints_file = os.path.join(
    #    nature_package_dir,
    #    'config',
    #    'waypoints_proving_ground_enu.yaml'
    #)
    
    waypoints_file = os.path.join(
        nature_package_dir,
        'config',
        'traverse_waypoints_dense.yaml'
    )
    
    nature_dir = get_package_share_directory('nature')
    base_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nature_dir, 'launch', 'base.launch_with_shutdown_no_perception.py')),
        launch_arguments={'waypoints_file': waypoints_file,
                          'robot_description': robot_desc,
                          'use_sim_time': 'True',
                          #'use_elevation': 'True',
                          'time_register_window': '0.09',
                          'use_global_path': 'True',
                          
                          'goal_dist': '4.0',
                          'num_paths': '41',
                          'vehicle_width': '1.5',
                          'max_steer_angle': '0.35',
                          'path_look_ahead': '20.0',

                          # 'slope_threshold': '0.75',
                          # 'use_registered': 'False',
                          # 'perception_warmup_time': '3.0',                          
                          # 'grid_res': '0.25',
                          # 'grid_dilate': 'False',
                          # 'grid_dilate_x': '0.25',
                          # 'grid_dilate_y': '0.25',
                          # 'grid_width': '500.0',
                          # 'grid_height': '500.0',
                          # 'grid_llx': '-250.0',
                          # 'grid_lly': '-250.0',
                          # 'stitch_lidar_points': 'False',
                          
                          'timeout': '500.0',
                          'vehicle_speed': vehicle_speed,
                          'steering_coefficient': '0.5',
                          'w_r': '0.001',
                          'w_s': '0.098',
                          'w_c': '0.001',
                          #'display_type': 'image',
                          'display_type': 'rviz'
                          }.items()
    )

    # some "global" MAVS parameters
    #scene_file = "cavs_proving_ground_sparse_trees.json"
    scene_file = "traversal_scene.json"
    #scene_file = "traversal_scene_no_trees.json"
    veh_file = "mrzr4_tires_low_gear.json"
    veh_scene_file = "traversal_scene_surface_only.json"
    #veh_scene_file = "traversal_scene_no_trees.json"
    use_registered_lidar = False
    env_params = {'env_params':
	        {"rain_rate": 0.0,
                "snow_rate": 0.0}
            }

    nature_ftte = launch_ros.actions.Node(
            package='nature',
            executable='nature_ftte_node',
            name='nature_ftte_node',
            output='screen',
            parameters=[
                {'use_registered':use_registered_lidar},
                {'vehicle_bumper_height': 0.25},
                {'use_loam':False},
                {'map_res': 0.5},
                {'map_width':200.0},
                {'map_length':200.0},
                {'fixed_map': False},
                {'default_traversability': 0.05},
                {'traversability_threshold': 0.05},
                {'slope_coeff': 0.0},
                {'soil_coeff': 0.0},
                {'veg_coeff': 0.9},
                {'veg_exponent': 0.5},
                {'roughness_coeff': 0.0},
                {'use_planes': True},
                {'show_timing': False},
                {'show_traversability': False},
                {'show_veg': False},
                {'show_ground': False},
                {'show_roughness': False},
                {'show_slope': False},
                {'save_plots': False},
            ]
        )

    mavs_vehicle = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_vehicle_node',
        name='mavs_vehicle_node',
        parameters=[
            {'use_sim_time':use_sim_time},
            {'scene_file': veh_scene_file},
            {'rp3d_vehicle_file': veh_file},
            {'soil_strength': 250.0},
            {'surface_type': "dry"},
            {'Initial_X_Position': 200.0},
            {'Initial_Y_Position': ystart},
            {'Initial_Heading': headstart},
            {'debug_camera': False},
            {'use_human_driver': False},
            {'scene_seed': global_scene_seed},
            {'publish_imu': True},
            env_params
        ],
        remappings=[
                (('/mavs/cmd_vel'), '/nature/cmd_vel'),
                (('/mavs/clock'), '/clock'),
                (('/mavs/odometry_true'), '/nature/odometry')
            ],
        output='screen',
        emulate_tty=True
    )
    
    mavs_gps = launch_ros.actions.Node(
            package='mavs-ros2',
            namespace='mavs',
            executable='mavs_gps_node',
            name='mavs_gps_node',
            parameters=[
                {'use_sim_time':use_sim_time},
                {'scene_file': scene_file},
                {'offset': [0.0,0.0,2.0]},
                {'orientation': [1.0, 0.0, 0.0, 0.0]},
                {'origin_lla': [33.475884, -88.787965, 70.0]},
                {'display': False},
                {'gps_type': "dual band"},
                {'update_rate_hz': 1.0},
                {'scene_seed': global_scene_seed},
                {'vehicle_files': [veh_file]},
                env_params
            ],
            remappings=[
                (('/mavs/odometry_true'), '/nature/odometry')
            ],
            output='screen',
            emulate_tty=True
        )

    mavs_lidar = launch_ros.actions.Node(
            package='mavs-ros2',
            namespace='mavs',
            executable='mavs_lidar_node',
            name='mavs_lidar_node',
            parameters=[
                {'use_sim_time':use_sim_time},
                {'scene_file': scene_file},
                {'offset': [1.5,0.0,0.25]},
                {'orientation': [1.0, 0.0, 0.0, 0.0]},
                {'display': False},
                {'scene_seed': global_scene_seed},
                {'update_rate_hz': 10.0},
                {'vehicle_files': [veh_file]},
                {'lidar_type' : 'OS2'}, #//Options are: HDL-32E', 'HDL-64E', 'M8','OS1', 'OS1-16', 'OS2', 'LMS-291', 'VLP-16', 'RS32
                {'register_points' : use_registered_lidar},
                env_params
            ],
            remappings=[
                (('/mavs/lidar'), '/nature/points'),
                (('/mavs/odometry_true'), '/nature/odometry')
            ],
            output='screen',
            emulate_tty=True
        )
    
    mavs_camera = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_camera_node',
        name='mavs_camera_node',
        parameters=[
            {'use_sim_time':use_sim_time},
            {'scene_file': scene_file},
             {'vehicle_files': [veh_file]},
            {'camera_type': "rgb"},
            {'num_horizontal_pix': 960},
            {'num_vertical_pix': 540},
            {'horizontal_pixel_plane_size': 0.006222},
            {'vertical_pixel_plane_size': 0.0035},
            {'focal_length': 0.0035},
            {'pixel_sample_factor':2},
            {'color_saturation':1.05},
            {'color_temp':7500.0},
            {'pixel_sample_factor':2},
            {'offset': [1.0,0.0,0.5]},
            #{'offset': [1.5,0.0,0.25]},
            {'orientation': [1.0, 0.0, 0.0, 0.0]},
            {'render_shadows': True},
            {'display': False},
            {'scene_seed': global_scene_seed},
            {'update_rate_hz': 10.0},
            env_params
        ],
        remappings=[
                (('/mavs/odometry_true'), '/nature/odometry')
            ],
        output='screen',
        emulate_tty=True
    )
    
    # 1. Create a timestamped unique directory name for the bag
    #timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    #bag_name = f"mavs_davs_rosbag_{timestamp}"
     # 2. Define launch configurations
    #record_bag_arg = DeclareLaunchArgument(
    #    'record_bag',
    #    default_value='true',
    #    description='Set to "true" to log topics to an MCAP bag file.'
    #)
    
    # Configure the mcap recorder process
    mcap_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-s', 'mcap', 
            '-o', [bag_name],
            '--all'
        ],
        output='screen',
        #condition=IfCondition(LaunchConfiguration('record_bag'))
    )
    
    # Shut down the entire top-level launch when nature_global_path_node exits.
    # base_launch's internal EmitEvent(Shutdown()) only kills the sub-context;
    # we match by process name here in the parent to propagate the shutdown up.
    global_path_shutdown_handler = RegisterEventHandler(
        event_handler=EventHandler(
            matcher=lambda event: (
                isinstance(event, ProcessExited) and
                'nature_global_path_node' in event.action.name
            ),
            entities=[EmitEvent(event=LaunchShutdown(reason='global_path_node finished, shutting down top-level launch'))]
        )
    )

    launch_description = LaunchDescription([
        vehicle_speed_arg,
        bag_name_arg,
        mavs_vehicle,
        mavs_lidar,
        mavs_camera,
        mavs_gps,
        base_launch,
        nature_ftte,
        global_path_shutdown_handler,
        LogInfo(msg=["Starting MCAP recording session. Saving to: ", bag_name]),
        mcap_recorder
    ])

    return launch_description
