import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import sys
import os
import numpy as np
import pinocchio as pin
import time
import pandas as pd
import argparse
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


class XsensDataLoader:
    def __init__(self):
        self.sheet = {
            "seg_pos": "Segment Position",
            "seg_ori": "Segment Orientation - Quat",
            "sensor_acc": "Sensor Free Acceleration",
            "sensor_ori": "Sensor Orientation - Quat",
            "sensor_mag": "Sensor Magnetic Field",
        }
        self.seg_list = [
            "Pelvis",
            "L5",
            "L3",
            "T12",
            "T8",
            "Neck",
            "Head",
            "Right Shoulder",
            "Right Upper Arm",
            "Right Forearm",
            "Right Hand",
            "Left Shoulder",
            "Left Upper Arm",
            "Left Forearm",
            "Left Hand",
            "Right Upper Leg",
            "Right Lower Leg",
            "Right Foot",
            "Right Toe",
            "Left Upper Leg",
            "Left Lower Leg",
            "Left Foot",
            "Left Toe",
        ]

    def load_seg_pose(self, path):
        # xlsx = pd.ExcelFile(path)
        # print("Available sheets:", xlsx.sheet_names)
        data_pos = pd.read_excel(path, sheet_name="Segment Position")
        data_ori = pd.read_excel(path, sheet_name="Segment Orientation - Quat")
        position = {}
        orientation = {}
        for seg in self.seg_list:
            position[seg] = data_pos[[seg + " x", seg + " y", seg + " z"]].to_numpy()
            orientation[seg] = data_ori[
                [seg + " q1", seg + " q2", seg + " q3", seg + " q0"]
            ].to_numpy()
        len = position["Pelvis"].shape[0]
        return len, position, orientation


class XsensIOMocapAdapter(Node):
    def __init__(self, xsens_data_path):
        super().__init__("XsensIOMocapAdapter")
        self.tf_publsiher = self.create_publisher(
            TFMessage, "/io_fusion/tf", QoSProfile(depth=1)
        )
        rate = 180
        self.cnt = 0
        self.load_tf_list()
        self.load_xsens_data(xsens_data_path)
        self.timer = self.create_timer(1.0 / rate, self.publish_transforms)

    def load_tf_list(self):
        self.tf_dic = [
            ["Pelvis", "link_Hips_Y", [0, 0, -1.5708]],
            # ["L5", [0, 0, 0]],
            ["L3", "link_Spine_Y", [0, 0, -1.5708]],
            ["T12", "link_Spine1_Y", [0, 0, -1.5708]],
            ["T8", "link_Spine2_Y", [0, 0, -1.5708]],
            ["Neck", "link_Neck1_Y", [0, 0, -1.5708]],
            ["Head", "link_Head_Y", [0, 0, -1.5708]],
            ["Right Shoulder", "link_RightShoulder_Y", [0, 0, -1.5708]],
            ["Right Upper Arm", "link_RightArm_Y", [-1.5708, 0, -1.5708]],
            ["Right Forearm", "link_RightForeArm_Y", [-1.5708, 0, -1.5708]],
            ["Right Hand", "link_RightHand_Y", [-1.5708, 0, -1.5708]],
            ["Left Shoulder", "link_LeftShoulder_Y", [0, 0, -1.5708]],
            ["Left Upper Arm", "link_LeftArm_Y", [1.5708, 0, -1.5708]],
            ["Left Forearm", "link_LeftForeArm_Y", [1.5708, 0, -1.5708]],
            ["Left Hand", "link_LeftHand_Y", [1.5708, 0, -1.5708]],
            ["Right Upper Leg", "link_RightUpLeg_Y", [0, 0, -1.5708]],
            ["Right Lower Leg", "link_RightLeg_Y", [0, 0, -1.5708]],
            ["Right Foot", "link_RightFoot_Y", [0, 0, -1.5708]],
            # ["Right Toe", [0, 0, 0]],
            ["Left Upper Leg", "link_LeftUpLeg_Y", [0, 0, -1.5708]],
            ["Left Lower Leg", "link_LeftLeg_Y", [0, 0, -1.5708]],
            ["Left Foot", "link_LeftFoot_Y", [0, 0, -1.5708]],
            # ["Left Toe", [0, 0, 0]],
            ["Left Hand", "link_LeftController", [1.5708, 0, -1.5708]],
            ["Right Hand", "link_RightController", [-1.5708, 0, -1.5708]],
        ]

    def load_xsens_data(self, xsens_data_path):
        xlsxloader = XsensDataLoader()
        self.length, self.seg_position, self.seg_orientation = xlsxloader.load_seg_pose(
            xsens_data_path
        )

    def publish_transforms(self):
        tfs = []
        for xsens_seg, io_seg, euler in self.tf_dic:
            R.from_quat(self.seg_orientation[xsens_seg][self.cnt])
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = io_seg
            p = self.seg_position[xsens_seg][self.cnt]
            tf.transform.translation = Vector3(x=p[0], y=p[1], z=p[2])
            q = (
                R.from_quat(self.seg_orientation[xsens_seg][self.cnt])
                * R.from_euler("XYZ", euler, degrees=False)
            ).as_quat()
            tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            tfs.append(tf)
        self.tf_publsiher.publish(TFMessage(transforms=tfs))
        self.cnt += 1
        if self.cnt >= self.length:
            self.cnt = 0


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("xsens_data_file")
    args = parser.parse_args()
    adapter = XsensIOMocapAdapter(args.xsens_data_file)
    rclpy.spin(adapter)
    adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
