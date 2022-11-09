from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_examples',
            executable='my_node',
            name='broadcaster1',
            # parameters=[
            #     {'turtlename': 'turtle1'}
            # ]
        ),
    ])