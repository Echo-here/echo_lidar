#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    channel_type = 'serial'
    serial_baudrate = 460800
    frame_id = 'laser'
    inverted = False
    angle_compensate = True
    scan_mode = 'Standard'

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Specifying usb port to connected lidar'),

        Node(
            package='echo_lidar',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=[
                '0', '0', '0.2',   # x, y, z (라이다 위치)
                '0', '0', '0',     # roll, pitch, yaw
                'base_link', frame_id  # 부모, 자식 프레임
            ]
        ),

    ])
