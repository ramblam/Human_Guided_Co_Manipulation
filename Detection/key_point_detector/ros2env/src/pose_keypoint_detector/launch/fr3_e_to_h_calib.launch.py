from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="st_cam_to_fr3_tf",
        arguments=[
            "-0.1271", "-0.3714", "0.9559",
            "0.0057", "-0.2740", "-0.0113", "-0.9617", # New Calculated Quaternion
            "fr3_link0",
            "st_cam_link"
        ],
        output="screen"
    )
    return LaunchDescription([
        static_tf
        ])