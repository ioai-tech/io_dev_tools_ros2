#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import threading
import os
import yaml
import time
import math

import geometry_msgs.msg
import sensor_msgs.msg


def sort_list(list_target, list_source, list_in):
    assert set(list_target) <= set(list_source)
    mapping = dict(zip(list_source, list_in))
    return np.array([mapping[i] for i in list_target])


def jpos_to_msg(q, name_list):
    msg = sensor_msgs.msg.JointState()
    msg.name = name_list
    msg.position = q
    return msg


def msg_to_jpos(msg, name_list):
    return sort_list(name_list, msg.name, msg.position)


class ControlROS(Node):
    def __init__(self, controller_file) -> None:
        super().__init__("robot_control_tool")

        with open(controller_file, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)
        self.control_joint = config["control_joints"]
        self.joint_name = self.control_joint.keys()
        self.joint_command = None
        self.joint_command_stamp = None
        self.joint_state = None
        self.init_flag = False
        self.msg = None
        self.dt = 1 / config["rate"]
        self.t = 0

        # ROS2
        self.rate = self.create_rate(config["rate"])
        self.joint_state_sub = self.create_subscription(
            sensor_msgs.msg.JointState,
            config["joint_state"],
            self.joint_state_callback,
            10,
        )
        self.ee_state_sub = self.create_subscription(
            geometry_msgs.msg.PoseArray, config["ee_state"], self.ee_state_callback, 10
        )

        self.joint_cmd_pub = self.create_publisher(
            sensor_msgs.msg.JointState, config["joint_cmd"], 10
        )

        update_robot_cmd = threading.Thread(target=self.update_robot_cmd)
        update_robot_cmd.daemon = True
        update_robot_cmd.start()

    def joint_state_callback(self, msg):
        self.joint_state = np.array(msg_to_jpos(msg, self.joint_name))
        if not self.init_flag:
            self.joint_command_stamp = list(self.joint_state)
            self.joint_command = list(self.joint_state)
            self.init_flag = True

    def ee_state_callback(self, msg):
        pass

    def update_robot_cmd(self):
        while rclpy.ok():
            if not self.init_flag:
                continue
            # pub joint command
            if self.msg is not None:
                self.msg.header.stamp = self.get_clock().now().to_msg()
                self.joint_cmd_pub.publish(self.msg)
            self.t += self.dt

            for idx, joint in enumerate(self.control_joint.values()):
                mag = joint["magnitude"]
                f = joint["frequence"]
                if joint["type"] == "sine":
                    self.joint_command[idx] = self.joint_command_stamp[
                        idx
                    ] + mag * math.sin(2 * math.pi * f * self.t)
                if joint["type"] == "step":
                    self.joint_command[idx] = self.joint_command_stamp[idx] + mag * (
                        int(self.t * f * 2) % 2 * -2 + 1
                    )
                if joint["type"] == "ramp":
                    self.joint_command[idx] = (
                        self.joint_command_stamp[idx]
                        + 2 * mag * abs((self.t * f % 1) / 0.5 - 1)
                        - mag
                    )
            self.msg = jpos_to_msg(self.joint_command, self.joint_name)

            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    config_file = os.path.join(os.path.dirname(__file__), "../config/" + "tool.yml")
    control_ros = ControlROS(config_file)

    rclpy.spin(control_ros)

    control_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
