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
from std_msgs.msg import Float64MultiArray


def l2r_hand(ori, trans):
    # y axis flip and x axis rotate 90 degree
    l2r_rot = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]])
    out_ori = R.from_matrix(l2r_rot @ ori.as_matrix() @ l2r_rot)
    out_trans = l2r_rot @ trans
    return out_ori, out_trans


class PicoIOMocapAdapter(Node):
    def __init__(self):
        super().__init__("PicoIOMocapAdapter")
        self.create_subscription(
            Float64MultiArray,
            "/io_teleop/vr_data_mocap",
            self.vr_data_callback,
            QoSProfile(depth=1),
        )
        self.tf_publsiher = self.create_publisher(
            TFMessage, "/io_fusion/tf", QoSProfile(depth=1)
        )
        self.seg_position = None
        self.seg_orientation = None
        self.load_tf_list()
        # self.timer = self.create_timer(1.0 / rate, self.publish_transforms)

    def load_tf_list(self):
        self.seg_list = [
            "Pelvis"  # 骨盆
            "LEFT_HIP"  # 左侧臀部
            "RIGHT_HIP"  # 右侧臀部
            "SPINE1"  # 脊柱
            "LEFT_KNEE"  # 左腿膝盖
            "RIGHT_KNEE"  # 右腿膝盖
            "SPINE2"  # 脊柱
            "LEFT_ANKLE"  # 左脚踝
            "RIGHT_ANKLE"  # 右脚踝
            "SPINE3"  # 脊柱
            "LEFT_FOOT"  # / 左脚
            "RIGHT_FOOT"  # / 右脚
            "NECK"  # / 颈部
            "LEFT_COLLAR"  # / 左侧锁骨
            "RIGHT_COLLAR"  # / 右侧锁骨
            "HEAD"  # / 头部
            "LEFT_SHOULDER"  # / 左肩膀
            "RIGHT_SHOULDER"  # / 右肩膀
            "LEFT_ELBOW"  # / 左手肘
            "RIGHT_ELBOW"  # / 右手肘
            "LEFT_WRIST"  # / 左手腕
            "RIGHT_WRIST"  # / 右手腕
            "LEFT_HAND"  # / 左手
            "RIGHT_HAND"  # 右手
        ]
        self.tf_dic = [
            ["Pelvis", "link_Hips_Y", [0, 0, 0]],
            ["LEFT_HIP", "link_LeftUpLeg_Y", [0, 0, 0]],
            ["RIGHT_HIP", "link_RightUpLeg_Y", [0, 0, 0]],
            ["SPINE1", "link_Spine_Y", [0, 0, 0]],
            ["LEFT_KNEE", "link_LeftLeg_Y", [0, 0, 0]],
            ["RIGHT_KNEE", "link_RightLeg_Y", [0, 0, 0]],
            ["SPINE2", "link_Spine1_Y", [0, 0, 0]],
            ["LEFT_ANKLE", "link_LeftFoot_Y", [0, 0, 0]],
            ["RIGHT_ANKLE", "link_RightFoot_Y", [0, 0, 0]],
            ["SPINE3", "link_Spine2_Y", [0, 0, 0]],
            ["LEFT_FOOT", "link_left_toe", [0, 0, 0]],
            ["RIGHT_FOOT", "link_right_toe", [0, 0, 0]],
            ["NECK", "link_Neck1_Y", [0, 0, 0]],
            ["LEFT_COLLAR", "link_LeftShoulder_Y", [0, 0, 0]],
            ["RIGHT_COLLAR", "link_RightShoulder_Y", [0, 0, 0]],
            ["HEAD", "link_Head_Y", [0, 0, 0]],
            ["LEFT_SHOULDER", "link_LeftArm_Y", [0, 1.5708, 0]],
            ["RIGHT_SHOULDER", "link_RightArm_Y", [0, -1.5708, 0]],
            ["LEFT_ELBOW", "link_LeftForeArm_Y", [0, 1.5708, 0]],
            ["RIGHT_ELBOW", "link_RightForeArm_Y", [0, -1.5708, 0]],
            ["LEFT_WRIST", "link_left_wrist", [0, 1.5708, 0]],
            ["RIGHT_WRIST", "link_right_wrist", [0, -1.5708, 0]],
            ["LEFT_HAND", "link_LeftHand_Y", [0, 1.5708, 0]],
            ["RIGHT_HAND", "link_RightHand_Y", [0, -1.5708, 0]],
        ]

    def vr_data_callback(self, msg):
        tfs = []
        for idx, (pico_seg, io_seg, euler) in enumerate(self.tf_dic):
            p_pico = np.array(msg.data[idx * 7 + 1 : idx * 7 + 4])
            q_pico = R.from_quat(msg.data[idx * 7 + 4 : idx * 7 + 8])
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = io_seg
            q, p = l2r_hand(q_pico, p_pico)
            tf.transform.translation = Vector3(x=p[0], y=p[1], z=p[2])
            q = (q * R.from_euler("XYZ", euler)).as_quat()
            tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            tfs.append(tf)
        self.tf_publsiher.publish(TFMessage(transforms=tfs))


def main(args=None):
    rclpy.init(args=args)
    adapter = PicoIOMocapAdapter()
    rclpy.spin(adapter)
    adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
