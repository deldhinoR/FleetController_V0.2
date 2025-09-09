from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    drones = [
        {"id": 1, "fcu_url": "udp://:14541@127.0.0.1:14557", "tgt_system": 2, "offset": [0.0, 0.0, 0.0]},
        {"id": 2, "fcu_url": "udp://:14542@127.0.0.1:14557", "tgt_system": 3, "offset": [-5.0, 2.9, 0.0]},
        {"id": 3, "fcu_url": "udp://:14543@127.0.0.1:14557", "tgt_system": 4, "offset": [-5.0, -2.9, 0.0]},
    ]

    actions = []

    for drone in drones:
        ns = f"drone{drone['id']}"
        actions.append(
            ExecuteProcess(
        cmd=[
            "ros2", "run", "mavros", "mavros_node",
            "--ros-args",
            "-r", f"__ns:=/{ns}",
            "-p", f"fcu_url:={drone['fcu_url']}",
            "-p", f"target_system_id:={drone['tgt_system']}",
            "-p", f"system_id:={drone['id']}",
            "-p", "plugin_whitelist:=command,local_position,sys_status"
        ],
        output="screen"
    )
)


        # Convert offset list to string for ROS2 CLI
        offset_str = "[" + ",".join(str(v) for v in drone["offset"]) + "]"

        # Mission computer (drone_mc) for this drone
        actions.append(
            ExecuteProcess(
                cmd=[
                    # //debug//       "gnome-terminal", "--",
                    "ros2", "run", "px4_test_scripts", "drone_mc",
                    "--ros-args",
                    "-p", f"drone_id:={drone['id']}",
                    "-p", f"offset:={offset_str}"
                ],
                output="screen"
            )
        )
    actions.append(
        ExecuteProcess(
            cmd=[
                "gnome-terminal", "--",
                "ros2", "run", "px4_test_scripts", "ground_controller"
            ],
            output="screen"
        )
    )

    return LaunchDescription(actions)
