def generate_launch_description():

	file_path = get_package_share_path(imu)

	declare_model_path = DeclarelaunchArgument(
		name = 'model',
		default_value = join(file_path, 'src', 'simpleIMU.urdf'),
		description = 'Absolute path yo robot urdf file'
	)

	declare_rviz_arg = DeclarelaunchArgument(
		name = 'rvizconfig',
		default_value = join(file_path, 'rviz/config.rviz'),
		description = 'Absolute path to rviz config file'
	)

	doc = xacro.parse(open(LaunchConfiguration('declare_model_path')))
	xacro.process_doc(doc)
	xml_code = doc.toxml()

	

	return LaunchDescription([

	])