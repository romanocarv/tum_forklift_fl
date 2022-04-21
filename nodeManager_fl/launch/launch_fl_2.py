import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = "~/tele_ws/src/tum_forklift/nodeManager_fl/config/params_fl.yaml"#os.path.join(
        #get_package_share_directory('nodeManager_fl'),
        #'config',
        #'params_fl.yaml'
        #)

    #rcan = Node(
    #    package='rossocketcan_bridge',
    #    executable='roscan_bridge',
    #    respawn=True#,
    #    #parameters=[config]
    #)

    ard = Node(
        package='arduinoManager_fl',
        executable='arduinoManager_fl',
        respawn=True#,
        #parameters=[config]
    )

    hb_fl = Node(
        package='heartbeat_fl',
        executable='heartbeat_fl'#,
        #respawn=True#,
        #parameters=[config]
    )

    stman_fl = Node(
        package='streamManager_fl',
        executable='streamManager_fl',
        #respawn=True#,
        parameters=[config]
    )

    print(config)
    #ld.add_action(fl_fl)
    #ld.add_action(rcan)
    ld.add_action(ard)
    ld.add_action(hb_fl)
    ld.add_action(stman_fl)

    return ld
