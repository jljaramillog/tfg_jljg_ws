from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='jljg_topics',
			executable='receptor',
			name='receptor'
		),
                Node(
                        package='jljg_topics',
                        executable='ackermann',
                        name='ackermann'
                ),
                Node(
                        package='jljg_topics',
                        executable='emisor_can',
                        name='emisor_can'
                ),
	])
