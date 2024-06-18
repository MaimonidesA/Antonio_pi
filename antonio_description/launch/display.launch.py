import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   # world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='antonio_description').find('antonio_description')
    default_model_path = os.path.join(pkg_share, 'src/description/antonio_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    #joint_state_publisher_node = launch_ros.actions.Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #    name='joint_state_publisher',
    #    arguments=[default_model_path],
    #    condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    #)
    #joint_state_publisher_gui_node = launch_ros.actions.Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    name='joint_state_publisher_gui',
    #    condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    #)
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )



    return launch.LaunchDescription([

        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'lidar_link',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        Node(
            package='antonio_description',
            executable='Wheels_odom_tf2_broadcaster',
            name='broadcaster1'
        ),


       # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
       #                                     description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'), 
                                   
        #joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node,

   
    ])