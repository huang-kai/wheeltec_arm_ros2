from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():

    # bringup_dir = get_package_share_directory('mini_mec_six_arm_moveit_config')
    # launch_dir = os.path.join(bringup_dir, 'launch')
    # voi_arm = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(launch_dir, 'demo.launch.py')),
    # )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'akmcar',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),
#the default mode is akm
        # launch_ros.actions.Node(
        #     condition=IfCondition(akmcar),
        #     package='turn_on_wheeltec_robot', 
        #     executable='wheeltec_robot_node', 
        #     output='screen',
        #     parameters=[{'usart_port_name': '/dev/wheeltec_controller',
        #         'serial_baud_rate': 115200,
        #         'robot_frame_id': 'base_footprint',
        #         'odom_frame_id': 'odom_combined',
        #         'cmd_vel': 'cmd_vel',
        #         'akm_cmd_vel': 'ackermann_cmd',
        #         'product_number': 0,}],
        #     remappings=[('/cmd_vel', 'cmd_vel'),]),

        # launch_ros.actions.Node(
        #     condition=IfCondition(akmcar),
        #     package='turn_on_wheeltec_robot', 
        #     executable='cmd_vel_to_ackermann_drive.py', 
        #     name='cmd_vel_to_ackermann_drive',
        #     ),
#the default mode is not akm
        launch_ros.actions.Node(
            # condition=UnlessCondition(akmcar),
            package='wheeltec_arm_pick', 
            executable='wheeltec_six_arm', 
            output='screen',
            parameters=[{'usart_port_name': '/dev/wheeltec_controller',
                'serial_baud_rate': 115200,
                'robot_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_combined',
                'cmd_vel': 'cmd_vel',
                'joint_num': 6,
                'product_number': 0,}],
        )

  ])
