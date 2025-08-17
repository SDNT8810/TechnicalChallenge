import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from nav2_common.launch import RewrittenYaml
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.substitutions import PathJoinSubstitution
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    pkg_name = 'sdnt_robot_simulation'
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sdnt_robot_simulation').find('sdnt_robot_simulation')
    # set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('tb3_model'))
    # tb3_desc_pkg = get_package_share_directory('turtlebot3_description')
    # default_model_path = os.path.join(tb3_desc_pkg, 'urdf/turtlebot3_waffle_pi.urdf')
    default_model_path = os.path.join(pkg_share, 'src/description/sdnt_robot_description.urdf')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')
    world_path=os.path.join(pkg_share, 'world/warehouse.world')
    map_path = os.path.join(pkg_share, 'maps/warehouse.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'sdnt_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.18'],
        output='screen'
    )
    spawn_tb3 = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': map_path}
        ],
        output='screen'
    )
    bringup_dir = get_package_share_directory('nav2_bringup')
    configured_params = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory(pkg_name), 'config/nav2_params.yaml'), root_key="", param_rewrites="", convert_types=True
        )
    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
            "map": map_path,
            "slam": "False",   
        }.items(),
    )

    # Localization (AMCL)
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        namespace='',
        parameters=[
            {'use_sim_time': True},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'global_frame_id': 'map'},
            {'scan_topic': '/scan'}
        ]
    )
    nav2_lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},  # Automatically transition nodes
            {'node_names': ['map_server']}  # Nodes to manage
        ]
    )
    static_transform_publisher_map2odom = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='false',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='map', default_value=os.path.join(pkg_share, 'maps/warehouse.yaml'),
                                            description='Absolute path to map file'),
        launch.actions.ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        # spawn_tb3,
        robot_localization_node,
        static_transform_publisher_map2odom,
        nav_node,
        rviz_node,
        amcl_node,
        map_server_node,
        nav2_lifecycle_manager,
    ])
