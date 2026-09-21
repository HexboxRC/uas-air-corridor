"""Wind geometry experiment, reusing the preloaded-aircraft visibility launch."""
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uas_sim.wind.prepare import prepare


def build(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    root = Path(get('run_root')).expanduser().resolve()
    share = Path(get_package_share_directory('uas_sim'))
    world, route = get('world'), get('route')
    actions = []
    drones, stagger = '2', '4'
    if get('enable_wind') == 'true':
        run = root / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        cfg = prepare(world, route, get('config'), run, policy=get('cell_policy'))
        cfg['controller']['policy'] = get('cell_policy')
        drones, stagger = str(cfg['drones']), str(cfg['spawn_stagger'])
        cfg['run_role'] = get('run_role')
        cfg['log_root'] = str(root / 'logs')
        runtime = run / 'runtime.json'
        runtime.write_text(json.dumps(cfg, indent=2))
        root.mkdir(parents=True, exist_ok=True)
        (root / 'latest_runtime.json').write_text(json.dumps(cfg, indent=2))
        world = cfg['world_file']
        plugin = Path(get_package_prefix('air_cell_wind')) / 'lib'
        if not (plugin / 'libAirCellWind.so').is_file():
            raise RuntimeError('Build air_cell_wind before launching')
        actions.append(SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join(filter(None, [str(plugin), os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))))
        bridges = ['/aircells/observations@std_msgs/msg/String[gz.msgs.StringMsg']
        for cell in cfg['cells']:
            base = '/aircells/' + cell['id']
            bridges += [base+'/wind_cmd@geometry_msgs/msg/Vector3]gz.msgs.Vector3d', base+'/width_cmd@std_msgs/msg/Float64]gz.msgs.Double']
        actions += [Node(package='ros_gz_bridge', executable='parameter_bridge', name='aircell_bridge', arguments=bridges, output='screen'),
                    Node(package='uas_sim', executable='wind_manager', name='wind_manager', arguments=['--runtime', str(runtime)] + ([] if get('monitor_gui') == 'true' else ['--no-gui']), output='screen')]
    # Apply middleware before creating either ROS process.
    actions.insert(0, SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'))
    actions.insert(1, SetEnvironmentVariable('ROS_DOMAIN_ID', '0'))
    actions.insert(2, SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch_folder/multi_drone_visible.launch.py')),
        launch_arguments={'world': world, 'route': route, 'px4_dir': get('px4_dir'),
                          'start_navigation': get('start_navigation'), 'show_viewer': 'false', 'drones': drones, 'spawn_stagger': stagger,
                          'render_engine': get('render_engine')}.items()))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('uas_sim'))
    args = {'world': str(Path.cwd()/'worlds/corridor.sdf'), 'route': str(Path.cwd()/'worlds/route.json'),
            'px4_dir': str(Path.home()/'Desktop/PX4-Autopilot'), 'config': str(share/'config/wind_demo.json'),
            'run_root': str(Path.cwd()/'wind_runs'), 'enable_wind': 'true', 'start_navigation': 'false',
            'cell_policy': 'adaptive', 'monitor_gui': 'false', 'render_engine': 'ogre2', 'run_role': 'evaluation'}
    choices = {'enable_wind': ['true','false'], 'start_navigation': ['true','false'], 'monitor_gui': ['true','false'],
               'cell_policy': ['adaptive','fixed_nominal','fixed_worst'], 'render_engine': ['ogre','ogre2'], 'run_role': ['calibration','evaluation']}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, **({'choices': choices[k]} if k in choices else {})) for k,v in args.items()] + [OpaqueFunction(function=build)])
