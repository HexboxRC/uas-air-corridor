"""Launch Gazebo, XRCE, two PX4 instances, and corridor navigation."""

import json
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PX4_DIR = os.path.expanduser('~/Desktop/PX4-Autopilot')
WORLD_PATH = os.path.expanduser('~/uas_ros2_ws/worlds/corridor.sdf')
ROUTE_PATH = os.path.expanduser('~/uas_ros2_ws/worlds/route.json')
N_DRONES = 2
CELL_STAGGER = 4
AIRFRAME = '4001'
MODEL = 'gz_x500'
AGENT_PORT = '8888'
SEPARATION = 60.0
VIEWER_PATH = os.path.expanduser('~/uas_ros2_ws/corridor_view.py')
SHOW_VIEWER = True

# Match these to the topics published by YOUR PX4/px4_msgs version.
STATUS_TOPIC = 'vehicle_status_v4'
POSITION_TOPIC = 'vehicle_local_position_v1'
ACK_TOPIC = 'vehicle_command_ack'


def plugin_dirs(root):
    out = []
    for directory, _, files in os.walk(root):
        if any(name.endswith('.so') for name in files):
            out.append(directory)
    return list(dict.fromkeys(out))


def generate_launch_description():
    build_dir = os.path.join(PX4_DIR, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(build_dir, 'bin', 'px4')
    if not os.path.isfile(px4_bin):
        raise RuntimeError(f'PX4 executable not found: {px4_bin}')

    world = ET.parse(WORLD_PATH).getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError('corridor.sdf must contain a named <world>.')
    world_name = world.get('name')
    existing_models = {model.get('name') for model in world.findall('model')}
    existing_models.update(
        include.findtext('name') for include in world.findall('include'))

    with open(ROUTE_PATH, encoding='utf-8') as handle:
        route = json.load(handle)
    lane0 = sorted(
        (cell for cell in route['cells'] if cell['lane'] == 0),
        key=lambda cell: cell['index'])
    if N_DRONES < 1 or CELL_STAGGER < 1:
        raise ValueError('N_DRONES and CELL_STAGGER must be positive.')
    if len(lane0) <= (N_DRONES - 1) * CELL_STAGGER:
        raise ValueError('Not enough lane-0 cells for the configured spawns.')

    names = [f'px4_{i}' for i in range(N_DRONES)]
    spawns = {
        name: [float(lane0[i * CELL_STAGGER]['x']),
               float(lane0[i * CELL_STAGGER]['y'])]
        for i, name in enumerate(names)
    }
    gz_res = os.pathsep.join(filter(None, [
        os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds'),
        os.path.dirname(WORLD_PATH),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    gz_plug = os.pathsep.join(filter(None, plugin_dirs(build_dir) + [
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))
    server_cfg = os.path.join(
        PX4_DIR, 'Tools', 'simulation', 'gz', 'server.config')

    actions = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plug),
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', server_cfg),
    ]
    # Stop the previous launch before running this one. Do not kill all PX4
    # processes automatically; other simulation sessions may be using them.
    actions.append(TimerAction(period=2.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r "{WORLD_PATH}"'}.items())]))
    actions.append(TimerAction(period=4.0, actions=[
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', AGENT_PORT],
            output='screen')]))
    actions.append(TimerAction(period=8.0, actions=[
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='clock_bridge', output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]))

    for i, name in enumerate(names):
        cell = lane0[i * CELL_STAGGER]
        env = {
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': world_name,
            'PX4_SYS_AUTOSTART': AIRFRAME,
            'PX4_SIM_MODEL': MODEL,
            'PX4_UXRCE_DDS_NS': name,
            'PX4_UXRCE_DDS_PORT': AGENT_PORT,
            'GZ_SIM_RESOURCE_PATH': gz_res,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plug,
            'GZ_SIM_SERVER_CONFIG_PATH': server_cfg,
        }
        if f'x500_{i}' in existing_models:
            # Bind only when this model is explicitly present in the world.
            # Its world pose must match the route's configured spawn point.
            env['PX4_GZ_MODEL_NAME'] = f'x500_{i}'
        else:
            env['PX4_GZ_MODEL_POSE'] = (
                f"{cell['x']:.2f},{cell['y']:.2f},0.3,0,0,"
                f"{cell.get('gz_yaw', 0.0):.6f}")
        print(f"{name}: expected model x500_{i}, spawn ENU=({cell['x']:.1f}, {cell['y']:.1f})")
        actions.append(TimerAction(period=7.0 + i * 4.0, actions=[
            ExecuteProcess(
                cmd=[px4_bin, '-i', str(i)], cwd=build_dir,
                output='screen', additional_env=env)]))

    for i, name in enumerate(names):
        cell = lane0[i * CELL_STAGGER]
        actions.append(TimerAction(period=25.0 + i * 2.0, actions=[
            Node(
                package='uas_sim', executable='corridor_nav',
                name=f'nav_{i}', output='screen',
                parameters=[{
                    'ns': name,
                    'target_system': i + 1,
                    'route_file': ROUTE_PATH,
                    'start_index': i * CELL_STAGGER,
                    'spawn_x': float(cell['x']),
                    'spawn_y': float(cell['y']),
                    'separation_m': SEPARATION,
                    'peers': names,
                    'peer_spawns_json': json.dumps(spawns),
                    'status_topic': STATUS_TOPIC,
                    'position_topic': POSITION_TOPIC,
                    'ack_topic': ACK_TOPIC,
                }])]))
    if SHOW_VIEWER:
        if not os.path.isfile(VIEWER_PATH):
            raise RuntimeError(f'Install the supplied viewer: {VIEWER_PATH}')
        actions.append(TimerAction(period=26.0, actions=[
            ExecuteProcess(
                cmd=['python3', VIEWER_PATH, '--route', ROUTE_PATH,
                     '--drones', str(N_DRONES), '--stagger', str(CELL_STAGGER),
                     '--position-topic', POSITION_TOPIC, '--status-topic', STATUS_TOPIC],
                output='screen')]))
    return LaunchDescription(actions)
