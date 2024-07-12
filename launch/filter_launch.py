from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,RegisterEventHandler
import launch
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():

    pcl_topic = LaunchConfiguration("pcl_topic")

    filter_node = Node(

            package="pcl_cpp",
            executable="VoxelGridAndOutlierRemoval",
            name="VoxelGridAndOutlierRemoval",
            output="screen",
            emulate_tty=True,
            remappings=[('/ouster/points', pcl_topic)],
            # parameters=[
            #     {"crop_box_x_min": -0.0},
            #     {"crop_box_x_max": 10.5},
            #     {"crop_box_y_min": -10.0},
        #     {"crop_box_y_max": 10.0},
            #     {"crop_box_z_min": -0.0},
            #     {"crop_box_z_max": 1.0},
            #     {"voxel_resolution": 0.2},
            #     {"setMean": 10.0},
            #     {"setStddevMulThresh": 0.5},
                
                
            # ]
            parameters=[
                {"crop_box_x_min": 0.5},
                {"crop_box_x_max": 50.0},
                {"crop_box_y_min": -50.0},
                {"crop_box_y_max": 50.0},
                {"crop_box_z_min": -5.0},
                {"crop_box_z_max": 5.0},
                {"voxel_resolution": 0.1},
                {"setMean": 10.0},
                {"setStddevMulThresh": 0.5},
                # {"rotation_angle_degrees": 180.0},
                {"rotation_quaternion_x": 0.0},
                {"rotation_quaternion_y": 1.0},
                {"rotation_quaternion_z": 0.0},
                {"rotation_quaternion_w": 0.0},
                
                
            ]
        )
    

    return LaunchDescription([
        DeclareLaunchArgument("pcl_topic", description="a pointcloud topic to process", default_value="ouster/points"),
        filter_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=filter_node,
                on_start=[launch.actions.LogInfo(msg=[f"\033[92mPoint_Cloud_filtering Node has started.\033[0m"])]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=filter_node,
                on_exit=[launch.actions.LogInfo(msg=[f"\033[92mPoint_Cloud_filtering Node has exited.\033[0m"])]
            )
        ),
        launch.actions.OpaqueFunction(function=lambda context: [
            launch.actions.LogInfo(msg=[f"\033[92mSelected pcl_Topic : {pcl_topic.perform(context)}\033[0m"]),
            launch.actions.LogInfo(msg=[f"\033[92mSelected Output_Topic : /filtered_points\033[0m"])
        ]),
    ])
