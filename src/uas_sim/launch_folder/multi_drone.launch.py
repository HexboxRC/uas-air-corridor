"""Launch the Denton -> Dallas corridor with N drones.

Starts Gazebo with the generated corridor world, the XRCE agent, N PX4
instances (each namespaced px4_0, px4_1, ...), and one corridor_nav node
per drone.

Run:
  python3 corridor_gen.py --miles 20 --lanes 1 --altitude 120 \
      --world ~/uas_ros2_ws/worlds/corridor.sdf \
      --route ~/uas_ros2_ws/worlds/route.json
  ros2 launch uas_sim multi_drone.launch.py
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ---- edit these -------------------------------------------------------
PX4_DIR    = os.path.expanduser('~/Desktop/PX4-Autopilot')
WORLD_PATH = os.path.expanduser('~/uas_ros2_ws/worlds/corridor.sdf')
ROUTE_PATH = os.path.expanduser('~/uas_ros2_ws/worlds/route.json')
N_DRONES   = 3
CELL_STAGGER = 4          # cells between drones at spawn
AIRFRAME   = '4001'
MODEL      = 'gz_x500'
AGENT_PORT = '8888'
SEPARATION = 60.0         # metres; session 4 makes this wind-dependent
# -----------------------------------------------------------------------


def plugin_dirs(root):
    out = []
    for r, _, files in os.walk(root):
        if any(f.endswith('.so') for f in files):
            out.append(r)
    return list(dict.fromkeys(out))


def generate_launch_description():
    build_dir = os.path.join(PX4_DIR, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(build_dir, 'bin', 'px4')

    gz_res = os.pathsep.join(filter(None, [
        os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds'),
        os.path.dirname(WORLD_PATH),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    gz_plug = os.pathsep.join(filter(None, plugin_dirs(build_dir) + [
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))
    server_cfg = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'server.config')

    with open(ROUTE_PATH) as f:
        route = json.load(f)
    lane0 = sorted([c for c in route['cells'] if c['lane'] == 0],
                   key=lambda c: c['index'])
    names = [f'px4_{i}' for i in range(N_DRONES)]

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plug),
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', server_cfg),
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'pkill -9 px4; pkill -9 MicroXRCEAgent; sleep 1; true'],
            output='screen'),
    ]

    actions.append(TimerAction(period=2.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {WORLD_PATH}'}.items())]))

    actions.append(TimerAction(period=4.0, actions=[
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', AGENT_PORT],
                       output='screen')]))

    for i in range(N_DRONES):
        cell = lane0[i * CELL_STAGGER]
        actions.append(TimerAction(period=7.0 + i * 4.0, actions=[
            ExecuteProcess(
                cmd=[px4_bin, '-i', str(i)],
                cwd=build_dir,
                output='screen',
                additional_env={
                    'PX4_GZ_STANDALONE': '1',
                    'PX4_SYS_AUTOSTART': AIRFRAME,
                    'PX4_SIM_MODEL': MODEL,
                    'PX4_GZ_MODEL_NAME': f'x500_{i}',
                    'PX4_GZ_MODEL_POSE':
                        f"{cell['x']:.2f},{cell['y']:.2f},0.3,0,0,{cell['yaw']:.4f}",
                    'PX4_UXRCE_DDS_NS': names[i],
                    'GZ_SIM_RESOURCE_PATH': gz_res,
                    'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plug,
                    'GZ_SIM_SERVER_CONFIG_PATH': server_cfg,
                })]))

    for i in range(N_DRONES):
        cell = lane0[i * CELL_STAGGER]
        actions.append(TimerAction(period=25.0 + i * 2.0, actions=[
            Node(package='uas_sim', executable='corridor_nav',
                 name=f'nav_{i}', output='screen',
                 parameters=[{
                     'ns': names[i],
                     'route_file': ROUTE_PATH,
                     'start_index': i * CELL_STAGGER,
                     'spawn_x': float(cell['x']),
                     'spawn_y': float(cell['y']),
                     'separation_m': SEPARATION,
                     'peers': names,
                 }])]))

    actions.append(TimerAction(period=8.0, actions=[
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]))

    return LaunchDescription(actions)