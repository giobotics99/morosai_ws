#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def make_lidar_group(ns, name, port, baud, frame, invert, angle_comp, scan_mode):
    return GroupAction([
        PushRosNamespace(ns),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name=name,
            output='screen',
            respawn=True,         
            respawn_delay=2.0,
            parameters=[{
                'channel_type': 'serial',
                'serial_port': port,
                'serial_baudrate': baud,
                'frame_id': frame,
                'inverted': invert,
                'angle_compensate': angle_comp,
                'scan_mode': scan_mode
            }],
            # remappings=[('/scan', 'scan')],  # opzionale, già ok così
        ),
    ])

def generate_launch_description():
    # === LIDAR 1 (FRONT) ===
    serial_port1 = LaunchConfiguration(
        'serial_port1',
        # default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_8518796e58ffb144875d33c7ca8065af-if00-port0' #lidar front ufficio
        # default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_6ddeb21a642a9c48be07ae66e837a4df-if00-port0'
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f333c9419ca7234b9f2cfdddac5a97a8-if00-port0'
    )
    serial_baudrate1 = LaunchConfiguration('serial_baudrate1', default='256000')
    frame_id1 = LaunchConfiguration('frame_id1', default='lidar_front_link')
    inverted1 = LaunchConfiguration('inverted1', default='false')
    angle_comp1 = LaunchConfiguration('angle_compensate1', default='true')
    scan_mode1 = LaunchConfiguration('scan_mode1', default='Sensitivity')
    ns1 = LaunchConfiguration('ns1', default='lidar_front')
    name1 = LaunchConfiguration('name1', default='sllidar_front')

    # === LIDAR 2 (REAR) ===
    serial_port2 = LaunchConfiguration(
        'serial_port2',
        # default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f3c7f819204a8b4681723ed792308877-if00-port0' #lidar rear ufficio
        # default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_a4e887758722a544970fdf170f4b6bb6-if00-port0'
         default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_fcab49ffcd27784eb3648dfeefe635af-if00-port0'
    )
    serial_baudrate2 = LaunchConfiguration('serial_baudrate2', default='256000')
    frame_id2 = LaunchConfiguration('frame_id2', default='lidar_rear_link')
    inverted2 = LaunchConfiguration('inverted2', default='false')
    angle_comp2 = LaunchConfiguration('angle_compensate2', default='true')
    scan_mode2 = LaunchConfiguration('scan_mode2', default='Sensitivity')
    ns2 = LaunchConfiguration('ns2', default='lidar_rear')
    name2 = LaunchConfiguration('name2', default='sllidar_rear')

    return LaunchDescription([
        # Declare args LIDAR 1
        DeclareLaunchArgument('serial_port1', default_value=serial_port1),
        DeclareLaunchArgument('serial_baudrate1', default_value=serial_baudrate1),
        DeclareLaunchArgument('frame_id1', default_value=frame_id1),
        DeclareLaunchArgument('inverted1', default_value=inverted1),
        DeclareLaunchArgument('angle_compensate1', default_value=angle_comp1),
        DeclareLaunchArgument('scan_mode1', default_value=scan_mode1),
        DeclareLaunchArgument('ns1', default_value=ns1),
        DeclareLaunchArgument('name1', default_value=name1),

        # Declare args LIDAR 2
        DeclareLaunchArgument('serial_port2', default_value=serial_port2),
        DeclareLaunchArgument('serial_baudrate2', default_value=serial_baudrate2),
        DeclareLaunchArgument('frame_id2', default_value=frame_id2),
        DeclareLaunchArgument('inverted2', default_value=inverted2),
        DeclareLaunchArgument('angle_compensate2', default_value=angle_comp2),
        DeclareLaunchArgument('scan_mode2', default_value=scan_mode2),
        DeclareLaunchArgument('ns2', default_value=ns2),
        DeclareLaunchArgument('name2', default_value=name2),

        # Gruppo LIDAR 1 (FRONT)
        make_lidar_group(
            ns=ns1,
            name=name1,
            port=serial_port1,
            baud=serial_baudrate1,
            frame=frame_id1,
            invert=inverted1,
            angle_comp=angle_comp1,
            scan_mode=scan_mode1
        ),

        # Gruppo LIDAR 2 (REAR)
        make_lidar_group(
            ns=ns2,
            name=name2,
            port=serial_port2,
            baud=serial_baudrate2,
            frame=frame_id2,
            invert=inverted2,
            angle_comp=angle_comp2,
            scan_mode=scan_mode2
        ),
    ])


# ls -l /dev/serial/by-id PER VEDERE PORTA USB DEL SENSORE E CAMBIARE LAUNCH IN BASE A FRONT E REAR