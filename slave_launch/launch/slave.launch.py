# /user/bin/python3
# coding = utf-8

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') 
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    diablo_node = Node(
        package = 'diablo_ctrl',
        executable = 'diablo_ctrl_node',
        output = 'screen',
        )

    laser_node = Node(
        package = 'sllidar_ros2',
        executable = 'sllidar_node',
        name = 'sllidar_node',
        parameters=[{'channel_type':channel_type,
                     'serial_port': serial_port, 
                     'serial_baudrate': serial_baudrate, 
                     'frame_id': frame_id,
                     'inverted': inverted, 
                     'angle_compensate': angle_compensate}],
        output = 'screen',
        )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=['0', '0', '0.25', '0', '0', '0', 'base_link', 'laser'],
        )

    odom_node = Node(
        package='diablo_odom',
        executable='odom_node',
        output = 'screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        use_sim_time_arg,
        diablo_node,
        laser_node,
        tf_node,
        odom_node
    ])
