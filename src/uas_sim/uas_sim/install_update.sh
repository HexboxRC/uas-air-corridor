#!/usr/bin/env bash
# Run after stopping the old launch and closing its Gazebo window.
set -e
bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
corridor_ws="$HOME/uas_ros2_ws"
package_dir="$corridor_ws/src/uas_sim"
if [ ! -f "$package_dir/uas_sim/corridor_nav.py" ]; then
  echo "Cannot find $package_dir/uas_sim/corridor_nav.py. No files changed."
  exit 1
fi
if ! python3 -c 'import tkinter' 2>/dev/null; then
  echo "Install the viewer dependency, then run this installer again:"
  echo "sudo apt install python3-tk"
  exit 1
fi
backup_dir="$(mktemp -d "$corridor_ws/backup_visuals_XXXXXXXX")"
mkdir -p "$corridor_ws/worlds" "$package_dir/launch"
for name in corridor_gen.py corridor_view.py start_corridor.sh; do
  if [ -f "$corridor_ws/$name" ]; then
    cp -p "$corridor_ws/$name" "$backup_dir/$name"
  fi
done
for name in corridor.sdf route.json; do
  if [ -f "$corridor_ws/worlds/$name" ]; then
    cp -p "$corridor_ws/worlds/$name" "$backup_dir/$name"
  fi
done
if [ -f "$package_dir/launch/multi_drone.launch.py" ]; then
  cp -p "$package_dir/launch/multi_drone.launch.py" "$backup_dir/"
fi
cp "$bundle_dir/corridor_gen.py" "$bundle_dir/corridor_view.py" \
   "$bundle_dir/start_corridor.sh" "$corridor_ws/"
cp "$bundle_dir/corridor.sdf" "$bundle_dir/route.json" "$corridor_ws/worlds/"
cp "$bundle_dir/multi_drone.launch.py" "$package_dir/launch/"
source /opt/ros/humble/setup.bash
source "$corridor_ws/install/setup.bash"
cd "$corridor_ws"
colcon build --packages-select uas_sim --symlink-install
echo "Installed. Previous files are in: $backup_dir"
echo "Start the updated simulation with: bash ~/uas_ros2_ws/start_corridor.sh"
