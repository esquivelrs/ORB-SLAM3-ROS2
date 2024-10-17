from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

package_dir = get_package_share_directory("orb_slam3_ros2_wrapper")

CONFIG_COMMON_DIR = PathJoinSubstitution([package_dir, "config"])


def generate_launch_description():
    params = [PathJoinSubstitution([CONFIG_COMMON_DIR, "slam.config.yaml"])]

    vocabulary_file_path = "/home/ros/ws/4dradar_camera_ws/src/general/ORB-SLAM3-ROS2/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    config_file_path = "/home/ros/ws/4dradar_camera_ws/src/general/ORB-SLAM3-ROS2/orb_slam3_ros2_wrapper/params/ark_mono.yaml"

    orb_slam3_node = Node(
        package="orb_slam3_ros2_wrapper",
        executable="mono",
        output="screen",
        namespace="/tractor/camera/front",
        arguments=[vocabulary_file_path, config_file_path],
        remappings=[
            (
                "/tractor/camera/front/camera/image_raw",
                "/tractor/camera/front/image_raw",
            )
        ],
        parameters=[
            params,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([orb_slam3_node])
