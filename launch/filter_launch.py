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
                {"crop_box_x_min": -0.0},
                {"crop_box_x_max": 10.5},
                {"crop_box_y_min": -5.0},
                {"crop_box_y_max": 5.0},
                {"crop_box_z_min": -1.0},
                {"crop_box_z_max": 1.0},
                {"voxel_resolution": 0.2},
                {"setMean": 10.0},
                {"setStddevMulThresh": 0.5},
                
                
            ]
        )
    ])