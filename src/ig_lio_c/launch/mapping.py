import os

from ament_index_python.packages import get_package_share_directory

from launch import conditions
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace

################### user configure parameters for ros2 start ###################
xfer_format   = 4    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 20.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = '47MDL330010038'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():

    ig_lio_c_dir = get_package_share_directory('ig_lio_c')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = os.path.join(ig_lio_c_dir, 'config', 'loca_parameters.yaml')
    bringup_dir = get_package_share_directory('rm_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')
    linefit_ground_segmentation_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    slam_params_file = LaunchConfiguration('slam_params_file')

    segmentation_params_file = os.path.join(linefit_ground_segmentation_dir, 'launch', 'segmentation_params.yaml')

    serial_config = os.path.join(
        get_package_share_directory('rm_serial_driver'), 'config', 'serial_driver.yaml')

    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=config,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value= os.path.join(bringup_dir,'map', '1_map.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rm_navigation"),
                                   'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    ig_lio_c_node = Node(
        package='ig_lio_c',
        executable='map_builder_node', 
        name='map_builder_node',
        output='screen',
        parameters=[configured_params],
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    # Nodes launching commands
    node_start_cmd = Node(
            package='linefit_ground_segmentation_ros',
            executable='ground_segmentation_node',
            output='screen',
            parameters=[segmentation_params_file])

    occupancy_grid_converter = Node(
        package='ig_lio_c',
        executable='occupancy_grid_converter',
        name='occupancy_grid_converter',
        output='screen',
        parameters=[configured_params],
    )

    imu_complementary_filter_node = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ],
                remappings=[
                	('/imu/data_raw', '/livox/imu'),
                ]
            )
    
    pointcloud_to_laserscan_node = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  ['/segmentation/obstacle']),
                        ('scan',  ['/scan'])],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': -0.40,
                'max_height': 0.1,
                'angle_min': -3.14159,# -M_PI/2 #0.39269875
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.4,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            # name='pointcloud_to_laserscan'
        )
    
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_mt',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
        #     condition=IfCondition(slam),
        #     launch_arguments={'namespace': namespace,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'use_respawn': use_respawn,
        #                       'params_file': params_file}.items()),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir,
        #                                                'localization_launch.py')),
        #     condition=IfCondition(PythonExpression(['not ', slam])),
        #     launch_arguments={'namespace': namespace,
        #                       'map': map_yaml_file,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': params_file,
        #                       'use_composition': use_composition,
        #                       'use_respawn': use_respawn,
        #                       'container_name': 'nav2_container'}.items()),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py'))
        ),                      
    ])
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    

    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_config],
    )


    ld = LaunchDescription([DeclareLaunchArgument('namespace', default_value='',
                            description='Top-level namespace'),
                            DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation (Gazebo) clock if true')])
    
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    

    ld.add_action(ig_lio_c_node)
    ld.add_action(livox_driver)
    ld.add_action(node_start_cmd)
    ld.add_action(imu_complementary_filter_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(stdout_linebuf_envvar)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    ld.add_action(rm_serial_driver_node)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(occupancy_grid_converter)



    return ld