from rosbags.highlevel import AnyReader
from tf2_msgs.msg import TFMessage
from rclpy.serialization import deserialize_message
from mcap.reader import make_reader
import rosbag2_py

import argparse
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation as R


class SmplAdapter:
    def __init__(self, old_data=False, pitch_correction=False):
        self.root = "link_Hips_Y"
        self.seg_dic = {
            "link_Hips_Y": ["base_link", [-1.5708, 0, 3.1416]],
            "link_LeftUpLeg_Y": ["link_Hips_Y", [-1.5708, 0, 3.1416]],
            "link_RightUpLeg_Y": ["link_Hips_Y", [-1.5708, 0, 3.1416]],
            "link_Spine_Y": ["link_Hips_Y", [-1.5708, 0, 3.1416]],
            "link_LeftLeg_Y": ["link_LeftUpLeg_Y", [-1.5708, 0, 3.1416]],
            "link_RightLeg_Y": ["link_RightUpLeg_Y", [-1.5708, 0, 3.1416]],
            "link_Spine1_Y": ["link_Spine_Y", [-1.5708, 0, 3.1416]],
            "link_LeftFoot_Y": ["link_LeftLeg_Y", [-1.5708, 0, 3.1416]],
            "link_RightFoot_Y": ["link_RightLeg_Y", [-1.5708, 0, 3.1416]],
            "link_Spine2_Y": ["link_Spine1_Y", [-1.5708, 0, 3.1416]],
            "link_LeftToe_Y": ["link_LeftFoot_Y", [-1.5708, 0, 3.1416]],
            "link_RightToe_Y": ["link_RightFoot_Y", [-1.5708, 0, 3.1416]],
            "link_Neck_Y": ["link_Spine2_Y", [-1.5708, 0, 3.1416]],
            "link_LeftShoulder_Y": ["link_Spine2_Y", [-1.5708, 0, 3.1416]],
            "link_RightShoulder_Y": ["link_Spine2_Y", [-1.5708, 0, 3.1416]],
            "link_Head_Y": ["link_Spine2_Y", [-1.5708, 0, 3.1416]],
            "link_LeftArm_Y": ["link_LeftShoulder_Y", [-1.5708, 0, 1.8708]],
            "link_RightArm_Y": ["link_RightShoulder_Y", [-1.5708, 0, -1.8708]],
            "link_LeftForeArm_Y": ["link_LeftArm_Y", [-1.5708, 0, 1.5708]],
            "link_RightForeArm_Y": ["link_RightArm_Y", [-1.5708, 0, -1.5708]],
            "link_LeftHand_Y": ["link_LeftForeArm_Y", [-1.5708, 0, 1.5708]],
            "link_RightHand_Y": ["link_RightForeArm_Y", [-1.5708, 0, -1.5708]],
        }
        for seg in self.seg_dic:
            self.seg_dic[seg][1] = R.from_euler("XYZ", self.seg_dic[seg][1])
        self.old_data = old_data
        self.pitch_correct = pitch_correction
        self.root_correct_R = R.from_quat([0, 0, 0, 1])

    def extract_tf_from_bag(self, bag_path_str):
        """
        从ROS2 bag文件中提取TF数据
        """
        from rosbags.typesys import get_typestore, Stores

        bag_path = Path(bag_path_str)
        tf_data = {}

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            for connection, _, data in reader.messages():
                if connection.topic != "/io_fusion/tf":
                    continue

                try:
                    # 解析TF消息
                    tf_msg = reader.deserialize(data, connection.msgtype)
                    self._parse_tf_data(tf_msg, tf_data)
                except Exception as e:
                    print(f"❌ 解析TF消息失败: {e}")
                    continue
        return tf_data, tf_data[self.root]["frame_count"]

    def extract_tf_from_mcap(self, mcap_path_str):
        """
        MCAP文件读取
        """

        mcap_path = Path(mcap_path_str)
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)

            # 获取文件信息
            summary = reader.get_summary()
            print(
                f"文件信息: {summary.statistics.message_count} 条消息，{len(summary.channels)} 个通道"
            )
            tf_data = {}
            for schema, channel, message in reader.iter_messages():
                if channel.topic != "/io_fusion/tf":
                    continue
                try:
                    tf_msg = deserialize_message(message.data, TFMessage)
                    self._parse_tf_data(tf_msg, tf_data)
                except Exception as e:
                    print(f"❌ 解析失败 :{e}")
        return tf_data, tf_data[self.root]["frame_count"]

    def convert_tf_to_smpl(self, tf_data, frame_count):
        # 初始化SMPL-X参数
        N = frame_count
        print("N: ", N)
        smpl_data = {
            "trans": np.zeros((N, 3), dtype=np.float32),  # 根关节平移
            "root_orient": np.zeros((N, 3), dtype=np.float32),  # 根关节旋转
            "pose_body": np.zeros((N, 63), dtype=np.float32),  # 身体姿态 (21关节×3)
            "poses": np.zeros((N, 165), dtype=np.float32),  # 身体+root姿态 (55关节×3)
            "betas": np.zeros(10, dtype=np.float32),  # 体型参数
            "mocap_frame_rate": np.float32(120),
            "gender": "male",
            "surface_model_type": "smplx",
        }
        smpl_data["betas"] = np.array(
            [
                0.57962339,
                -0.73276458,
                0.33848802,
                -0.28104899,
                0.37962434,
                -0.51072268,
                1.31321042,
                0.62262523,
                0.16257167,
                -0.81578137,
                -0.29703843,
                -0.06884368,
                1.12179023,
                0.15773111,
                0.19307245,
                0.70862296,
            ]
        )

        # convert_tf_to_smpl_coordinates
        for idx, (seg, (parent_seg, convert)) in enumerate(self.seg_dic.items()):
            if seg in ["link_LeftToe_Y", "link_RightToe_Y"]:
                continue
            # root
            if seg is self.root:
                smpl_data["trans"] = np.array(tf_data[seg]["translations"])
                rotvec = [
                    (rot * convert).as_rotvec() for rot in tf_data[seg]["rotations"]
                ]
                smpl_data["root_orient"] = np.array(rotvec)
                smpl_data["poses"][:, 0:3] = np.array(rotvec)
            # body
            else:
                rot_list = []
                for j_idx in range(N):
                    rot_child = tf_data[seg]["rotations"][j_idx] * convert
                    rot_parent = (
                        tf_data[parent_seg]["rotations"][j_idx]
                        * self.seg_dic[parent_seg][1]
                    )
                    rot = (rot_parent.inv() * rot_child).as_rotvec()
                    rot_list.append(rot)
                rotvec = np.array(rot_list)
                smpl_data["poses"][:, idx * 3 : idx * 3 + 3] = rotvec
        return smpl_data

    def _parse_tf_data(self, tf_msg, tf_data):
        for transform in tf_msg.transforms:
            frame_name = transform.child_frame_id
            header_time = transform.header.stamp

            # 时间戳转换为秒
            time_sec = header_time.sec + header_time.nanosec * 1e-9

            # 提取平移
            translation = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )

            # 提取旋转（四元数）
            rotation = R.from_quat(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )

            # for old data preprocess
            if self.old_data:
                if frame_name == self.root:
                    root_rot = rotation
                else:
                    rotation = root_rot * rotation
            # for pitch correction
            if self.pitch_correct:
                if frame_name == self.root:
                    if frame_name not in tf_data:
                        pitch = rotation.as_euler("ZYX", degrees=False)[2]
                        print("Pitch correction angle: ", pitch)
                        self.root_correct_R = R.from_euler(
                            "XYZ", [-pitch, 0, 0], degrees=False
                        )
                    rotation = rotation * self.root_correct_R

            # 存储数据
            if frame_name not in tf_data:
                tf_data[frame_name] = {
                    "timestamps": [],
                    "translations": [],
                    "rotations": [],
                    "frame_count": 0,
                }

            tf_data[frame_name]["timestamps"].append(time_sec)
            tf_data[frame_name]["translations"].append(translation)
            tf_data[frame_name]["rotations"].append(rotation)
            tf_data[frame_name]["frame_count"] += 1


def convert_from_rosbag_to_smpl(adapter, file_path, smpl_path):
    """
    主函数: 将io数据文件转换为SMPL-X npz文件
    """

    print("*** 开始转换 SMPL-X npz ***")
    print("=" * 60)
    # 步骤1: 提取TF数据
    print("\n1 提取TF数据...")
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"File not found: {file_path}")
    ext = file_path.suffix.lower()
    # if ext in [".db3", ".bag"]:
    if ext == ".mcap":
        print("mcap 转换")
        tf_data, frame_cnt = adapter.extract_tf_from_mcap(file_path)

    else:
        print("ros2bag 转换")
        tf_data, frame_cnt = adapter.extract_tf_from_bag(file_path)
        # raise ValueError(
        #     f"不支持的文件格式: '{ext}'\n"
        #     f"文件: {file_path}\n"
        #     f"支持的格式: .db3, .bag, .mcap"
        # )

    if not tf_data:
        raise ValueError("未提取到TF数据")

    # 步骤2: 生成SMPL-X参数
    print("\n2 生成SMPL-X参数...")
    smpl_data = adapter.convert_tf_to_smpl(tf_data, frame_cnt)
    # save_full_data_to_file(smpl_data, str(smpl_path) + ".txt")
    print(f"\n3 保存为npz文件: {smpl_path}")
    np.savez_compressed(smpl_path, **smpl_data)

    # 验证生成的文件
    print("\n *** 转换完成! ***")
    print("=" * 60)


def save_full_data_to_file(data, filename, max_elements_per_array=10000):
    """
    将完整数据保存到文本文件

    参数:
        data: 数据字典
        filename: 输出文件名
        max_elements_per_array: 每个数组最大保存元素数
    """
    with open(filename, "w", encoding="utf-8") as f:

        def write_value(obj, indent=0):
            """递归写入值"""
            indent_str = " " * indent

            if isinstance(obj, np.ndarray):
                f.write(
                    f"{indent_str}np.ndarray(shape={obj.shape}, dtype={obj.dtype})\n"
                )

                if obj.size > 0:
                    f.write(f"{indent_str}统计信息:\n")
                    f.write(f"{indent_str}  最小值: {obj.min():.6f}\n")
                    f.write(f"{indent_str}  最大值: {obj.max():.6f}\n")
                    f.write(f"{indent_str}  平均值: {obj.mean():.6f}\n")
                    f.write(f"{indent_str}  标准差: {obj.std():.6f}\n")

                    if obj.size <= max_elements_per_array:
                        f.write(f"{indent_str}完整数据:\n")
                        np.set_printoptions(
                            threshold=np.inf, linewidth=200, precision=6
                        )
                        f.write(f"{indent_str}{obj}\n")
                    else:
                        f.write(f"{indent_str}数据预览（前1000个元素）:\n")
                        preview = obj.flat[:1000]
                        f.write(f"{indent_str}{preview}\n")
                        f.write(
                            f"{indent_str}... 还有 {obj.size - 1000} 个元素未显示\n"
                        )

            elif isinstance(obj, dict):
                for k, v in obj.items():
                    f.write(f"{indent_str}{k}:\n")
                    write_value(v, indent + 2)

            elif isinstance(obj, (list, tuple)):
                f.write(f"{indent_str}{type(obj).__name__} 长度: {len(obj)}\n")
                for i, item in enumerate(obj[:100]):  # 最多显示100个
                    f.write(f"{indent_str}[{i}]: ")
                    if isinstance(item, (np.ndarray, dict, list, tuple)):
                        f.write("\n")
                        write_value(item, indent + 4)
                    else:
                        f.write(f"{item}\n")
                if len(obj) > 100:
                    f.write(f"{indent_str}... 还有 {len(obj) - 100} 个元素\n")

            elif isinstance(obj, np.generic):
                f.write(f"{indent_str}{obj}\n")

            else:
                f.write(f"{indent_str}{obj}\n")

        # 写入头部
        f.write("=" * 80 + "\n")
        f.write(f"数据完整导出 - {filename}\n")
        f.write("=" * 80 + "\n\n")

        # 写入数据
        for key, value in data.items():
            f.write(f"[{key}]\n")
            f.write("-" * 40 + "\n")
            write_value(value, indent=2)
            f.write("\n" + "=" * 80 + "\n\n")

    print(f"✅ 完整数据已保存到: {filename}")
    print(f"   使用命令查看: less {filename} 或 cat {filename} | head -n 100")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_path", type=str, help="ros2bag路径")
    parser.add_argument("--output_path", default=None, type=str, help="smpl路径")
    parser.add_argument(
        "--is_old_data", default=False, type=bool, help="是否为旧版本数据"
    )
    parser.add_argument(
        "--is_pitch_correction", default=False, type=bool, help="root pitch是否修正"
    )
    args = parser.parse_args()
    file_path = args.file_path
    output_path = args.output_path
    if output_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = file_path + f"_smplx_animation_{timestamp}.npz"
    adapter = SmplAdapter(args.is_old_data, args.is_pitch_correction)
    convert_from_rosbag_to_smpl(adapter, file_path, output_path)
