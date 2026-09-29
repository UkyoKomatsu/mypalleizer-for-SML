from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    root = Path('/opt/sim')
    robot_file = root / 'urdf' / 'robot.urdf'
    description = robot_file.read_text(encoding='utf-8')
    return LaunchDescription([
        ExecuteProcess(cmd=[
            'gzserver', '--verbose', str(root / 'worlds' / 'lego.world'),
            '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
        ], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}],
             output='screen'),
        TimerAction(period=6.0, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-file', str(robot_file), '-entity', 'mypalletizer_260_pi'],
                 output='screen'),
        ]),
        TimerAction(period=16.0, actions=[
            Node(package='controller_manager', executable='spawner.py',
                 arguments=['joint_state_broadcaster', '-c', '/controller_manager'], output='screen'),
            Node(package='controller_manager', executable='spawner.py',
                 arguments=['arm_controller', '-c', '/controller_manager'], output='screen'),
            Node(package='controller_manager', executable='spawner.py',
                 arguments=['gripper_controller', '-c', '/controller_manager'], output='screen'),
        ]),
        ExecuteProcess(cmd=['python3', str(root / 'scripts' / 'socket_bridge.py')],
                       output='screen'),
    ])
