from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument('motor_id', default_value=[1,2,3,4,5,6]),
        # DeclareLaunchArgument('udp_receive_port', default_value=[1,2,3,4,5,6]),
        # DeclareLaunchArgument('udp send_port', default_value=[1,2,3,4,5,6]),
        # DeclareLaunchArgument('publish_frequency', default_value=[1,2,3,4,5,6]),
        Node(
            package='pi3hat_moteus_control',
            executable='moteus_control',
            namespace="",
            name='moteus_control',
            output='screen',
            # parameters=[{'publish_frequency': LaunchConfiguration('publish_frequency')}],
            # Launch the node with root access (GPIO) in a shell
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        ),
        Node(
            package='pi3hat_moteus_control',
            executable='udp_connector.py',
            namespace="",
            name='udp_connector',
            output='screen',
            shell=True,
        ),
    ])