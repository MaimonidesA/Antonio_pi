from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)

    # First sensor node
    parameters_file_path_1 = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node_Left.yaml')
    xsens_mti_node_1 = Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node_1',
            namespace='Left_imu',
            output='screen',
            parameters=[parameters_file_path_1],
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
    ld.add_action(xsens_mti_node_1)

    # Second sensor node
    parameters_file_path_2 = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node_right.yaml')
    xsens_mti_node_2 = Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node_2',
            namespace='right_imu',
            output='screen',
            parameters=[parameters_file_path_2],
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
    ld.add_action(xsens_mti_node_2)

    return ld