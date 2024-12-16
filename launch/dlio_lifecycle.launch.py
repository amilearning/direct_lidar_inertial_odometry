from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition  # Add this import
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')

    # Set default arguments
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/ouster/points')
    imu_topic = LaunchConfiguration('imu_topic', default='/ouster/imu')

    # Define arguments    
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

    # Lifecycle DLIO Odometry Node
    dlio_odom_node = LifecycleNode(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        name='dlio_odom_node',
        namespace='',
        output='log',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # Lifecycle DLIO Mapping Node
    dlio_map_node = LifecycleNode(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        name='dlio_map_node',
        namespace='',
        output='log',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )

    # Event to transition Odometry Node to 'configure'
    configure_dlio_odom = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == dlio_odom_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Event to transition Odometry Node to 'activate'
    activate_dlio_odom = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == dlio_odom_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    # Event Handler: Transition Mapping Node to 'configure' and 'activate' after Odometry Node is Active
    configure_and_activate_dlio_map = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=dlio_odom_node,
            start_state='active',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == dlio_map_node,
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == dlio_map_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([        
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        dlio_odom_node,
        dlio_map_node,
        configure_dlio_odom,  # Configure Odometry Node
        activate_dlio_odom,   # Activate Odometry Node
        configure_and_activate_dlio_map,  # Configure and Activate Mapping Node After Odometry
    ])
