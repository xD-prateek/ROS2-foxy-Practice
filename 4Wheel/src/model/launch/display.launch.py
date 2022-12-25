from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.launch_description_sources import PythonLaunchDescriptionSource


from os.path import join
import xacro


def generate_launch_description():

	default_world_file_name = 'simple_world.world'  #edit this
	default_urdf_file_name = 'urdf_eg3.urdf'   #edit this
	package_name = 'model'   #edit this



	file_path = get_package_share_path(package_name)
	# pkg_gazebo_ros = get_package_share_path('gazebo_ros')

	declare_gui_arg = DeclareLaunchArgument(
		name='gui',
		default_value='false', 
		choices=['true', 'false'], 
		description='Flag to enable joint_state_publisher_gui'
	)

	# declare_namespace = DeclareLaunchArgument(
	# 	name='use_namespace',
	# 	default_value='',
	# 	description='Whether to apply namespace to navigation stack'
	# )

	declare_model_path = DeclareLaunchArgument(
		name='model',
		default_value=join(file_path, 'urdf', default_urdf_file_name),
		description='Absolute path to robot urdf file'	
	)

	declare_rviz_arg = DeclareLaunchArgument(
		name='rvizconfig',
		default_value=join(file_path, 'rviz/config.rviz'),
		description='Absolute path to rviz config file'
	)

	declare_use_sim_time = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='true',
		description='Use simulation (GAZEBO) clock if true'
	)

	# declare_simulator_cmd = DeclareLaunchArgument(
	# 	name='headless',
	# 	default_value='False',
	# 	description='Whether to start simulator or not'
	# )

	declare_world_path = DeclareLaunchArgument(
		name='world',
		default_value=join(file_path, 'worlds', default_world_file_name),
		description='Full path to world model file to load'
	)

	# robot_description = ParameterValue(
	# 	Command(['xacro ', LaunchConfiguration('model')]),
	# 	value_type=str
	# )

	xacro_file = join(file_path, 'urdf', default_urdf_file_name)
	doc = xacro.parse(open(xacro_file))
	xacro.process_doc(doc)
	xml_code = doc.toxml()

	# gazebo = IncludeLaunchDescription(
 # 				PythonLaunchDescriptionSource([join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
 #    )

	gazebo = ExecuteProcess(
				cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
				output='screen'
    )

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description':xml_code, 'use_sim_time':LaunchConfiguration('use_sim_time')}]
	)

	# depending on the gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
	joint_state_publisher_node = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		condition=UnlessCondition(LaunchConfiguration('gui'))
	)

	joint_state_publisher_gui_node = Node(
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		condition=IfCondition(LaunchConfiguration('gui'))
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', LaunchConfiguration('rvizconfig')],
	)

	spawn_entity_node = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		name='urdf_spawnner',
		output='screen',
		arguments=["-entity", xml_code, "-topic", "robot_description"]
	)

	return LaunchDescription([
		declare_gui_arg,
		declare_model_path,
		declare_rviz_arg,
		declare_use_sim_time,
		declare_world_path,
		joint_state_publisher_node,
		joint_state_publisher_gui_node,
		gazebo,
		rviz_node,
		robot_state_publisher_node,
		spawn_entity_node
	])