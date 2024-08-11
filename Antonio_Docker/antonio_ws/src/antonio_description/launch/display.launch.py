import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
   # world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='antonio_description').find('antonio_description')
    default_model_path = os.path.join(pkg_share, 'src/description/antonio_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    # rplidar s2
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar_S1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000') #for s2 is 1000000
    frame_id = LaunchConfiguration('frame_id', default='Floor_scan')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')
    # Xsens
    xsens_parameters_file_path_1 = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node_Left.yaml')
    xsens_parameters_file_path_2 = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node_right.yaml')


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rplidar_A1_Node = launch_ros.actions.Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/lidar_A1',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'lidar_link',
            'topic_name': 'scan',
            'inverted': False,
            'angle_compensate': True }],  
    )
    rplidar_s2_Node = launch_ros.actions.Node(
            package='sllidar_ros2',
            executable='sllidar_node', #  /dev/ttyUSB0
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode }],
            output='screen')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    xsens_mti_node_1 = launch_ros.actions.Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node_1',
            namespace='Left_imu',
            output='screen',
            parameters=[xsens_parameters_file_path_1],
            remappings=[
                ('/filter/free_acceleration', '/Left_imu/filter/free_acceleration'),
                ('/filter/quaternion', '/Left_imu/filter/quaternion'),
                ('/filter/twist', '/Left_imu/filter/twist'),
                ('/imu/acceleration', '/Left_imu/imu/acceleration'),
                ('/imu/angular_velocity', '/Left_imu/imu/angular_velocity'),
                ('/imu/data', '/Left_imu/imu/data'),
                ('/imu/dq', '/Left_imu/imu/dq'),
                ('/imu/dv', '/Left_imu/imu/dv'),
                ('/imu/mag', '/Left_imu/imu/mag'),
                ('/imu/time_ref', '/Left_imu/imu/time_ref'),
                ('/pressure', '/Left_imu/pressure'),
                ('/status', '/Left_imu/status'),
                ('/temperature', '/Left_imu/temperature'),
                ('/tf', '/Left_imu/tf'),
                ('/utctime', '/Left_imu/utctime')
            ]
            )
    xsens_mti_node_2 = launch_ros.actions.Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node_2',
            namespace='right_imu',
            output='screen',
            parameters=[xsens_parameters_file_path_2],
            remappings=[
                ('/filter/free_acceleration', '/right_imu/filter/free_acceleration'),
                ('/filter/quaternion', '/right_imu/filter/quaternion'),
                ('/filter/twist', '/right_imu/filter/twist'),
                ('/imu/acceleration', '/right_imu/imu/acceleration'),
                ('/imu/angular_velocity', '/right_imu/imu/angular_velocity'),
                ('/imu/data', '/right_imu/imu/data'),
                ('/imu/dq', '/right_imu/imu/dq'),
                ('/imu/dv', '/right_imu/imu/dv'),
                ('/imu/mag', '/right_imu/imu/mag'),
                ('/imu/time_ref', '/right_imu/imu/time_ref'),
                ('/pressure', '/right_imu/pressure'),
                ('/status', '/right_imu/status'),
                ('/temperature', '/right_imu/temperature'),
                ('/tf', '/right_imu/tf'),
                ('/utctime', '/right_imu/utctime')
            ]
            )



    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        launch.actions.DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        launch.actions.DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        launch.actions.DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        launch.actions.DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        launch.actions.DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        launch.actions.DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'), 
                                   
        #joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        #robot_state_publisher_node,
        #robot_localization_node, 
        xsens_mti_node_1,
        xsens_mti_node_2,
        rplidar_s2_Node,
        rplidar_A1_Node,
        #rviz_node,

   
    ])