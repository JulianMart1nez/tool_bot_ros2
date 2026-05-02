# System snapshot — captured 2026-05-02

A live snapshot of the running ROS 2 graph and TF tree taken with the full
voice → detect_zone → go_to → MoveIt 2 stack up. The robot was offline
(IP 192.168.1.234 unreachable) but every node still came up — the
`ufactory_driver` keeps retrying its hardware connection while
publishing the rest of its graph.

## What was running at capture time

| Terminal | Command | Nodes contributed |
|---|---|---|
| T1 | `xarm5_moveit_with_table.launch.py` | `move_group`, `controller_manager`, `xarm5_traj_controller`, `robot_state_publisher`, `joint_state_publisher`, `ufactory_driver`, `ufactory_robot_hw`, `rviz2`, `static_transform_publisher`, `moveit_simple_controller_manager` |
| T2 | `depth_camera.launch.py` | `gripper_cam/depth_camera` (RealSense), `fine_localization`, `detect_zone`, `gripper_camera_tf` (static TF) |
| T3 | `voice_command_node` | `voice_command_node` |
| T4 | `go_to` (keyboard variant) | `go_to` |
| T5 | `debug_overlay` + `rqt_image_view` | `debug_overlay`, `rqt_gui_cpp_node_*` |

## Files

| File | What it is |
|---|---|
| `nodes.txt` | `ros2 node list` |
| `topics.txt` | `ros2 topic list -t` |
| `services.txt` | `ros2 service list -t` |
| `actions.txt` | `ros2 action list` |
| `node_info/*.txt` | `ros2 node info <name>` per node |
| `ros_graph.gv` | Graphviz DOT — node ↔ topic graph (the rqt_graph equivalent) |
| `ros_graph.png` / `.pdf` | Rendered ros_graph |
| `tf_frames.gv` | Graphviz DOT for the TF tree (output of `view_frames`) |
| `tf_frames.png` / `.pdf` | Rendered TF tree |
| `build_rqt_graph.py` | Re-runnable builder for `ros_graph.*` from `node_info/*.txt` |
| `captured_at.txt` | UTC timestamp of capture |

## How to refresh

With the stack running:

```bash
cd ~/tool_bot_ros2/docs/system_snapshot
source /opt/ros/jazzy/setup.bash
date -u +%Y-%m-%dT%H:%M:%SZ > captured_at.txt
ros2 node list    --no-daemon --spin-time 5 | sort -u  > nodes.txt
ros2 topic list   --no-daemon --spin-time 5 -t | sort  > topics.txt
ros2 service list --no-daemon --spin-time 5 -t | sort  > services.txt
ros2 daemon start && sleep 3
ros2 action list                                | sort  > actions.txt
mkdir -p node_info
for n in $(cat nodes.txt | grep -v WARNING); do
  safe=$(echo "$n" | tr '/' '_' | sed 's/^_//')
  ros2 node info "$n" > "node_info/${safe}.txt" 2>&1
done
python3 build_rqt_graph.py
dot -Tpng ros_graph.gv -o ros_graph.png
dot -Tpdf ros_graph.gv -o ros_graph.pdf
ros2 run tf2_tools view_frames -o tf_frames
dot -Tpng tf_frames.gv -o tf_frames.png
```

`--no-daemon --spin-time 5` is critical — the cached daemon view often
misses half the nodes; the longer DDS spin window catches them all.
