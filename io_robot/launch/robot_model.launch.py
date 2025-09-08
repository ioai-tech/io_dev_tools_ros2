from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def get_urdf_content_from_yaml(urdf_file):
    try:
        urdf_dir = os.path.dirname(urdf_file)
        print(f"Loading URDF from: {urdf_file}")
        try:
            with open(urdf_file, "r") as urdf:
                robot_description_content = urdf.read()
                robot_description_content = robot_description_content.replace(
                    "file://", f"file://{urdf_dir}/"
                )
            return robot_description_content
        except Exception as e:
            print(f"Failed to read URDF file: {e}")
            return None
    except Exception as e:
        print(f"YAML load failed! {e}")
        return None


def launch_setup(context, *args, **kwargs):
    """在运行时执行的函数"""
    # 获取启动参数的实际值
    namespace = context.launch_configurations["robot_namespace"]
    robot_name_val = context.launch_configurations["robot_name"]
    use_sim_val = context.launch_configurations["use_sim"]
    use_gui_val = context.launch_configurations["use_gui"]
    urdf_file = context.launch_configurations["urdf_file"]

    # 获取包路径（用于配置文件）
    pkg_share = get_package_share_directory("io_robot")

    # 构建配置文件路径（YAML文件在io_robot包内）
    sim_config_file = os.path.join(pkg_share, "config", robot_name_val, "sim_ros.yml")
    print("sim config file: ", sim_config_file)

    # 获取URDF文件名
    robot_description_content = get_urdf_content_from_yaml(urdf_file)

    # 构建RViz配置文件路径（RViz配置在io_robot包内）
    rviz_config_file = os.path.join(pkg_share, "config", robot_name_val, "rviz.rviz")

    # 创建节点列表
    nodes_to_start = [
        # 设置仿真时间参数
        # SetParameter(name="use_sim_time", value=use_sim_val),
        GroupAction(
            [
                PushRosNamespace(namespace),
                # RViz节点
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz",
                    arguments=["-d", rviz_config_file],
                    remappings=[
                        ("/robot_description", f"/{namespace}/robot_description"),
                        ("/tf", f"/{namespace}/tf"),
                        ("/tf_static", f"/{namespace}/tf_static"),
                    ],
                ),
                # 机器人状态发布器 - 通过参数传递URDF内容
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    condition=IfCondition(
                        PythonExpression([f"{use_sim_val} or {use_gui_val}"])
                    ),
                    parameters=[
                        {
                            "robot_description": robot_description_content,
                        }
                    ],
                    remappings=[
                        ("/tf", f"/{namespace}/tf"),
                        ("/tf_static", f"/{namespace}/tf_static"),
                    ],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    condition=IfCondition(
                        PythonExpression([f"not {use_sim_val} and not {use_gui_val}"])
                    ),
                    parameters=[{"robot_description": robot_description_content}],
                    # 立即退出，避免占用资源
                    on_exit=lambda context: None,
                ),
                # 非仿真模式下的GUI关节发布器
                Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    name="joint_state_publisher_gui",
                    condition=IfCondition(
                        PythonExpression([f"not {use_sim_val} and {use_gui_val}"])
                    ),
                    # remappings=[("/joint_states", f"{namespace}/joint_states")],
                ),
                # 仿真模式下的机器人仿真节点
                Node(
                    package="io_robot",
                    executable="robot_sim_ros2.py",
                    name="robot_sim_v2",
                    output="screen",
                    condition=IfCondition(use_sim_val),
                    arguments=[
                        "--config_file",
                        sim_config_file,
                        "--urdf_file",
                        urdf_file,
                    ],
                    # parameters 用于传递 ROS2 参数，不是命令行参数
                    # parameters=[{"some_ros_param": "value"}],
                ),
            ]
        )
    ]

    return nodes_to_start


def generate_launch_description():
    # 定义启动参数
    robot_namespace = DeclareLaunchArgument(
        "robot_namespace", default_value="io_teleop", description="机器人命名空间"
    )

    robot_name = DeclareLaunchArgument(
        "robot_name", default_value="G1", description="机器人名称"
    )

    robot_path = DeclareLaunchArgument(
        "robot_path",
        default_value="",  # 你可以设置一个默认路径
        description="URDF文件的基础路径",
    )

    use_gui = DeclareLaunchArgument(
        "use_gui", default_value="True", description="是否使用GUI"
    )

    use_sim = DeclareLaunchArgument(
        "use_sim", default_value="True", description="是否使用仿真模式"
    )

    return LaunchDescription(
        [
            robot_namespace,
            robot_name,
            robot_path,
            use_gui,
            use_sim,
            OpaqueFunction(function=launch_setup),
        ]
    )
