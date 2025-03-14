import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        # Declare IP address argument
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.102',  # Change to default UR10e IP
            description='IP address of the UR10e robot'
        ),

        # UR10e driver node
        launch_ros.actions.Node(
            package='ur_robot_driver',
            executable='ur_robot_driver_node',
            name='ur10e_driver',
            output='screen',
            parameters=[{
                "robot_ip": LaunchConfiguration('robot_ip'),
                "use_sim_time": False
            }]
        ),

        # Data collection node
        launch_ros.actions.Node(
            package='ur10e_data_collector',
            executable='data_collector',
            name='ur10e_data_collector',
            output='screen'
        )
    ])
