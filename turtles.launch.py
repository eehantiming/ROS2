from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()

	turtlesim = Node(
		package='turtlesim',
		executable='turtlesim_node'
		)

	spawner = Node(
		package='my_py_pkg',
		executable='turtle_spawner',
		parameters=[
			{'rate': 0.5}
		])

	controller = Node(
		package='my_py_pkg',
		executable='turtle_controller',
		parameters=[
			{'xmin': 3,
			'xp': 10,
			'anglep': 20}
		])

	ld.add_action(turtlesim)
	ld.add_action(spawner)
	ld.add_action(controller)
	return ld