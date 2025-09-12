from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    qgc = ExecuteProcess(
        cmd=["/home/hild/Downloads/QGroundControl-x86_64.AppImage"], output="screen"
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen"
    )

    gimbal_params = os.path.join(
        get_package_share_directory("aligner"), "cfg", "gimbal.yaml"
    )

    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={gimbal_params}",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            qgc,
            micro_xrce_agent,
            gz_ros_bridge,
        ]
    )

    # return LaunchDescription([

    #     Node(
    #         package='object_detector',
    #         executable='object_detector_node',
    #         output='screen',
    #     ),

    #     Node(
    #         package='aligner',
    #         executable='roacher',
    #         output='screen',
    #     ),

    #     Node(
    #         package='aligner',
    #         executable='aligner',
    #         name='yaw_aligner',
    #         output='screen',
    #         parameters=[
    #             PathJoinSubstitution([FindPackageShare('aligner'), 'cfg', 'params.yaml'])
    #         ]
    #     ),
    # ])
