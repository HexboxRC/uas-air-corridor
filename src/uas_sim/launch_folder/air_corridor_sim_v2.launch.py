import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# -----------------------------
# User paths
# -----------------------------
PX4_DIR = os.path.expanduser("~/Desktop/PX4-Autopilot")
WORLD_PATH = os.path.expanduser("~/Desktop/gazebo_worlds/test_world.sdf")

# -----------------------------
# PX4 settings
# -----------------------------
AIRFRAME = "4001"
MODEL = "gz_x500"

# Format: x, y, z, roll, pitch, yaw
SPAWN_POSE = "0,0,0.2,0,0,0"

AGENT_PORT = "8888"


def join_paths(*paths):
    return os.pathsep.join([p for p in paths if p])


def find_plugin_dirs(root_dir):
    plugin_dirs = []

    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".so"):
                plugin_dirs.append(root)
                break

    return list(dict.fromkeys(plugin_dirs))


def generate_launch_description():
    px4_build_dir = os.path.join(
        PX4_DIR,
        "build",
        "px4_sitl_default",
    )

    px4_bin = os.path.join(
        px4_build_dir,
        "bin",
        "px4",
    )

    px4_models = os.path.join(
        PX4_DIR,
        "Tools",
        "simulation",
        "gz",
        "models",
    )

    px4_worlds = os.path.join(
        PX4_DIR,
        "Tools",
        "simulation",
        "gz",
        "worlds",
    )

    custom_world_dir = os.path.dirname(WORLD_PATH)

    sim_gz_models = os.path.expanduser("~/.simulation-gazebo/models")
    sim_gz_worlds = os.path.expanduser("~/.simulation-gazebo/worlds")

    server_config = os.path.join(
        PX4_DIR,
        "Tools",
        "simulation",
        "gz",
        "server.config",
    )

    gz_resource_path = join_paths(
        px4_models,
        px4_worlds,
        custom_world_dir,
        sim_gz_models,
        sim_gz_worlds,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    px4_plugin_dirs = find_plugin_dirs(px4_build_dir)

    gz_plugin_path = join_paths(
        *px4_plugin_dirs,
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
    )

    cleanup = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "killall -q px4 MicroXRCEAgent gz gzserver gzclient || true",
        ],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {WORLD_PATH}",
        }.items(),
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=[
            "MicroXRCEAgent",
            "udp4",
            "-p",
            AGENT_PORT,
        ],
        output="screen",
    )

    px4 = ExecuteProcess(
        cmd=[
            px4_bin,
            "-i",
            "0",
        ],
        cwd=px4_build_dir,
        output="screen",
        additional_env={
            "PX4_GZ_STANDALONE": "1",
            "PX4_SYS_AUTOSTART": AIRFRAME,

            # PX4 spawns the Gazebo x500 model
            "PX4_SIM_MODEL": MODEL,

            # Spawn pose: x,y,z,roll,pitch,yaw
            "PX4_GZ_MODEL_POSE": SPAWN_POSE,

            # World name without .sdf
            "PX4_GZ_WORLD": "test_world",

            # Gazebo paths also passed into PX4
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GZ_SIM_SERVER_CONFIG_PATH": server_config,
        },
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription([
        cleanup,

        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_resource_path,
        ),

        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=gz_plugin_path,
        ),

        SetEnvironmentVariable(
            name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            value=gz_plugin_path,
        ),

        SetEnvironmentVariable(
            name="GZ_SIM_SERVER_CONFIG_PATH",
            value=server_config,
        ),

        TimerAction(
            period=2.0,
            actions=[
                gz_sim,
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                micro_xrce_agent,
            ],
        ),

        TimerAction(
            period=7.0,
            actions=[
                px4,
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                clock_bridge,
            ],
        ),
    ])