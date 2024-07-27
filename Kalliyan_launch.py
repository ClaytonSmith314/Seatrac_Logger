import launch
import launch_ros.actions
from datetime import datetime

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            name='modem'
        ),
        launch_ros.actions.Node(
            package='kalliyan_field_test',
            executable='kalliyan_pinger',
            name='kalliyan_pinger',
            parameters=[{'beacon_ids_to_ping': [2,3,4,5]}]
        ),
        launch_ros.actions.Node(
            package='kalliyan_field_test',
            executable='kalliyan_logger',
            name='kalliyan_logger'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', f'_kalliyan_folder/ros_bag/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}', '/modem_rec'],
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()