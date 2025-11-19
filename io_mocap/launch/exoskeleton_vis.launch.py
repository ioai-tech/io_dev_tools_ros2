from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 定义 URDF 文件路径
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("io_mocap"), "description", "blender_human_skeleton_v4.urdf"]
    )

    # 启动 robot_state_publisher
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="screen",
    #     parameters=[{"robot_description": Command(["xacro ", urdf_path])}],
    #     remappings=[
    #         ("/tf", "/io_fusion/tf"),
    #         # ("/tf_static", "/io_fusion/tf_static"),
    #     ],
    # )
    # 临时节点，仅用于设置 /robot_description 参数
    set_robot_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", urdf_path])}],
        # 立即退出，避免占用资源
        on_exit=lambda context: None,
    )
    # 启动 RViz2
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("io_mocap"), "rviz", "rviz_config_skeleton.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        remappings=[
            ("/tf", "/io_fusion/tf"),
        ],
    )

    # 启动 joint_state_publisher_gui（用于手动控制关节）
    # joint_state_publisher_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    # )

    return LaunchDescription(
        [
            # robot_state_publisher,
            #  joint_state_publisher_gui,
            set_robot_description,
            rviz_node,
        ]
    )
    # return LaunchDescription([set_robot_description, rviz_node])
