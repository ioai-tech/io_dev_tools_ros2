#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import JointState

import os
import threading
import yaml
import argparse
import time

from robot_sim_bullet import *


class RobotSimRos(Node):
    def __init__(self, config_file, urdf_file) -> None:
        SimEnv()
        with open(config_file, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)

        self.robot = RobotDemo(urdf_file, config.get("fixed_link", None))

        # ros
        super().__init__("Robot_Sim_ROS2")

        self.create_subscription(
            JointState,
            config["sub_topic"],
            self.joint_command_callback,
            QoSProfile(depth=1),
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            config["pub_topic"],
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            ),
        )
        self.robot.home()
        time.sleep(0.5)
        init_q = config.get("init_q", None)
        if init_q:
            print(init_q.values())
            print(init_q.keys())
            self.robot.move_arm(
                list(init_q.values()),
                [self.robot.jointName2IdDic[joint] for joint in init_q.keys()],
            )
        time.sleep(0.5)
        self.dt = 0.01
        update_robot_state = threading.Thread(target=self.update_robot_state)
        update_robot_state.daemon = True
        update_robot_state.start()

    def update_robot_state(self):
        while True:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            # print(self.get_clock().now().to_msg())
            joint_state.name = self.robot.getJointNameList()
            joint_state.position = self.robot.getJointStates()
            self.joint_state_pub.publish(joint_state)
            time.sleep(self.dt)

    def joint_command_callback(self, msg):
        list = self.robot.getJointNameList()
        # print("全部关节的名称:\n", list)
        # print("cmd对应关节名称:\n", msg.name)
        # print("cmd对应关节信息:\n", msg.position)
        # 获取cmd信息
        id_list, position_list = self.sort_list(list, msg.name, msg.position)
        # print("-----------\n",id_list, "-----\n", position_list)
        self.robot.move_arm(position_list, jointIdList=id_list)

    # cmd信息对齐
    def sort_list(self, list_target, list_source, list_in):
        jointNamelist = set(list_target) & set(list_source)
        assert jointNamelist != None
        mapping = dict(zip(list_source, list_in))
        jointIdlist = [self.robot.jointName2IdDic[i] for i in jointNamelist]
        return jointIdlist, [mapping[i] for i in jointNamelist]


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--config_file", type=str, required=True)
    parser.add_argument("--urdf_file", type=str, required=True)
    args, _ = parser.parse_known_args()
    control_ros = RobotSimRos(
        args.config_file,
        args.urdf_file,
    )
    rclpy.spin(control_ros)
    control_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    # rospy.init_node("robot_sim_ros_node")
    # config_file = None
    # try:
    #     config_file = rospy.get_param("config_file")
    # except KeyError:
    #     rospy.logwarn("config_file not found on the parameter server")
    # config_file = os.path.join(os.path.dirname(__file__), "../config/" + config_file)
    # rospy.spin()
