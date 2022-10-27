import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower'),
  ])