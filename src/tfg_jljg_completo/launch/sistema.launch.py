from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='tfg_jljg_completo',
			executable='receptor',
			name='receptor'
		),
                Node(
                        package='tfg_jljg_completo',
                        executable='ackermann',
                        name='ackermann'
                ),
                Node(
                        package='tfg_jljg_completo',
                        executable='emisor_can',
                        name='emisor_can'
                ),
	])
