"""
SEN771 AGVC Launch File
Starts: world generator → Gazebo Ionic + ros_gz_bridge + RViz2 + path planner

Launch arguments:
  obs_size:=6    Obstacle size in metres — all 15 obstacles will be this size.
                 Maximum allowed: 10 m  (default: 6 m)
  autostart:=true    Gazebo starts running immediately (default)
  autostart:=false   Gazebo starts paused — press Play in GUI to begin
  rviz:=true         Launch RViz2 alongside Gazebo
  run_planner:=false Launch without the planner node
"""

import os
import subprocess
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GENERATED_WORLD = '/tmp/sen771_world.sdf'


def generate_launch_description():
    pkg_sen771 = get_package_share_directory('sen771_agvc')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    rviz_file  = os.path.join(pkg_sen771, 'rviz', 'sen771.rviz')

    generator_script = os.path.join(pkg_sen771, 'scripts', 'generate_world.py')

    def launch_all(context):
        obs_size  = LaunchConfiguration('obs_size').perform(context)
        autostart = LaunchConfiguration('autostart').perform(context)

        # ── 1. Run world generator ───────────────────────────────────────────
        print(f'\n[sen771] Generating random world (obs_size={obs_size} m)...')
        result = subprocess.run(
            [sys.executable, generator_script, obs_size],
            check=False,
        )
        if result.returncode != 0:
            print('[sen771] WARNING: World generator failed — using fallback world.')
            world = os.path.join(pkg_sen771, 'worlds', 'sen771_world.sdf')
        else:
            world = GENERATED_WORLD

        # ── 2. Launch Gazebo with generated world ────────────────────────────
        gz_args = f'-r {world}' if autostart == 'true' else world
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        )
        return [gz_sim]

    # ── ros_gz_bridge ─────────────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/sen771_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/sen771_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/sen771_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        output='screen',
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # ── Path planner node ─────────────────────────────────────────────────────
    planner = Node(
        package='sen771_agvc',
        executable='planner',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('run_planner')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'obs_size', default_value='6',
            description='Obstacle size in metres (all 15 obstacles, max 10 m). Default: 6'),
        DeclareLaunchArgument('autostart',   default_value='true',
                              description='Start Gazebo immediately (true) or paused (false)'),
        DeclareLaunchArgument('rviz',        default_value='false',
                              description='Launch RViz2 alongside Gazebo'),
        DeclareLaunchArgument('run_planner', default_value='true',
                              description='Run the path planner node'),
        OpaqueFunction(function=launch_all),
        bridge,
        rviz,
        planner,
    ])
