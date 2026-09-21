"""Existing PX4 flight plus localized wind and changing cell side panels.

Run prepare.py and build.sh before this launch. Original uas_sim navigation
is used without modifications. Cell entry recommendations are NOT enforced.
"""

import json
import math
import tempfile
from pathlib import Path
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import (SetEnvironmentVariable, TimerAction,
                            DeclareLaunchArgument, OpaqueFunction,
                            UnsetEnvironmentVariable, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PX4_DIR = os.path.expanduser('~/Desktop/PX4-Autopilot')
LAB_DIR = Path(__file__).resolve().parent
RUNTIME_PATH = LAB_DIR / 'run' / 'runtime.json'
if not RUNTIME_PATH.is_file():
    raise RuntimeError('Run python3 prepare.py in the wind_cell_lab directory first.')
RUNTIME = json.loads(RUNTIME_PATH.read_text())
WORLD_PATH = RUNTIME['world_file']
ROUTE_PATH = RUNTIME['route_file']
N_DRONES = RUNTIME['drones']
CELL_STAGGER = RUNTIME['spawn_stagger']
AIRFRAME = '4001'
AGENT_PORT = '8888'
SEPARATION = 60.0
VIEWER_PATH = os.path.expanduser('~/uas_ros2_ws/corridor_view.py')
SHOW_VIEWER = False  # This experiment supplies its own cell monitor.

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


def build_actions(context):
    if not (LAB_DIR / 'build' / 'libAirCellWind.so').is_file():
        raise RuntimeError('Run bash build.sh with the Gazebo major version first.')
    sim_major = (LAB_DIR/'build'/'gazebo_major.txt').read_text().strip()
    if sim_major not in ('7', '8', '9'):
        raise RuntimeError('Unexpected Gazebo version in build metadata. Rebuild the plugin.')
    build_dir = os.path.join(PX4_DIR, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(build_dir, 'bin', 'px4')
    if not os.path.isfile(px4_bin):
        raise RuntimeError(f'PX4 executable not found: {px4_bin}')

    tree = ET.parse(WORLD_PATH)
    world = tree.getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError('corridor.sdf must contain a named <world>.')
    world_name = world.get('name')

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
    # Put aircraft into the initial world snapshot, rather than inserting
    # them after GUI startup. PX4 will bind to these exact model names.
    model_file = os.path.join(
        PX4_DIR, 'Tools', 'simulation', 'gz', 'models', 'x500', 'model.sdf')
    if not os.path.isfile(model_file):
        raise RuntimeError(f'x500 model file not found: {model_file}')
    for i in range(N_DRONES):
        model_name = f'x500_{i}'
        # Only replace the two intended aircraft if the source already has them.
        for model in list(world.findall('model')):
            if model.get('name') == model_name:
                world.remove(model)
        for include in list(world.findall('include')):
            if include.findtext('name') == model_name:
                world.remove(include)
        cell = lane0[i * CELL_STAGGER]
        yaw = cell.get('gz_yaw')
        if yaw is None:
            j = i * CELL_STAGGER
            a, b = (lane0[j], lane0[j + 1]) if j + 1 < len(lane0) else (lane0[j - 1], lane0[j])
            yaw = math.atan2(b['y'] - a['y'], b['x'] - a['x'])
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = model_file
        ET.SubElement(include, 'name').text = model_name
        ET.SubElement(include, 'pose').text = (
            f"{float(cell['x']):.6f} {float(cell['y']):.6f} 0.3 0 0 {float(yaw):.6f}")
    engine = LaunchConfiguration('render_engine').perform(context)
    gui = world.find('gui')
    if gui is None:
        raise RuntimeError('Expected the supplied corridor world to have a <gui> configuration.')
    scene = next((p for p in gui.findall('plugin')
                  if p.get('filename') in ('MinimalScene', 'GzScene3D')), None)
    if scene is None:
        raise RuntimeError('Expected a MinimalScene or GzScene3D GUI plugin.')
    engine_element = scene.find('engine')
    if engine_element is None:
        engine_element = ET.SubElement(scene, 'engine')
    engine_element.text = engine
    temporary = tempfile.TemporaryDirectory(prefix='uas_corridor_')
    run_world = os.path.join(temporary.name, 'corridor.sdf')
    tree.write(run_world, encoding='utf-8', xml_declaration=True)

    def cleanup(_context, *args, **kwargs):
        temporary.cleanup()
        return []

    gz_res = os.pathsep.join(filter(None, [
        os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'models'),
        os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds'),
        os.path.dirname(WORLD_PATH),
        os.path.dirname(RUNTIME['source_world']),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    gz_plug = os.pathsep.join(filter(None, plugin_dirs(build_dir) + [str(LAB_DIR / 'build'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))
    server_cfg = os.path.join(
        PX4_DIR, 'Tools', 'simulation', 'gz', 'server.config')

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        UnsetEnvironmentVariable('PX4_SIM_MODEL'),
        UnsetEnvironmentVariable('PX4_GZ_MODEL'),
        UnsetEnvironmentVariable('PX4_GZ_MODEL_POSE'),
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
            launch_arguments={'gz_args': f'--force-version {sim_major} -r -v 4 "{run_world}"'}.items())]))
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
            'PX4_GZ_MODEL_NAME': f'x500_{i}',
            'PX4_UXRCE_DDS_NS': name,
            'PX4_UXRCE_DDS_PORT': AGENT_PORT,
            'GZ_SIM_RESOURCE_PATH': gz_res,
            'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plug,
            'GZ_SIM_SERVER_CONFIG_PATH': server_cfg,
        }
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
                condition=IfCondition(LaunchConfiguration('start_navigation')),
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
    bridge_arguments = ['/aircells/observations@std_msgs/msg/String[gz.msgs.StringMsg']
    for cell in RUNTIME['cells']:
        base = '/aircells/' + cell['id']
        bridge_arguments += [base + '/wind_cmd@geometry_msgs/msg/Vector3]gz.msgs.Vector3d',
                             base + '/width_cmd@std_msgs/msg/Float64]gz.msgs.Double']
    actions.append(TimerAction(period=5.0, actions=[Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='aircells_bridge', output='screen', arguments=bridge_arguments)]))
    manager = ['python3', str(LAB_DIR/'cell_manager.py'), '--runtime', str(RUNTIME_PATH),
               '--policy', LaunchConfiguration('cell_policy').perform(context)]
    if LaunchConfiguration('monitor_gui').perform(context) == 'false':
        manager.append('--no-gui')
    actions.append(TimerAction(period=6.0, actions=[ExecuteProcess(cmd=manager, output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell_policy', default_value=RUNTIME['controller']['policy'],
            choices=['adaptive', 'fixed_nominal', 'fixed_worst']),
        DeclareLaunchArgument('monitor_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'start_navigation', default_value='false', choices=['true', 'false'],
            description='Start the existing corridor_nav nodes and enable automatic flight.'),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre2', choices=['ogre2', 'ogre'],
            description='GUI renderer; ogre is an optional WSL graphics compatibility test.'),
        OpaqueFunction(function=build_actions),
    ])
