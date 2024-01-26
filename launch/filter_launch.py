from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pcl_cpp",
            executable="VoxelGridAndOutlierRemoval",
            name="custom_VoxelGridAndOutlierRemoval",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pass_through_z_min": -0.0},
                {"pass_through_z_max": 0.5},
                {"pass_through_x_min": -1.0},
                {"pass_through_x_max": 1.0},
                {"pass_through_y_min": -1.0},
                {"pass_through_y_max": 1.0},
            ]
        )
    ])