import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import threading


class SimEnv:
    def __init__(self) -> None:
        gui_client = p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setGravity(0, 0, -9.81)
        physics_sim_thread = threading.Thread(target=self.physics_sim)
        physics_sim_thread.daemon = True
        physics_sim_thread.start()

    def physics_sim(self):
        p.setRealTimeSimulation(1)
        while True:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)


class RobotDemo:
    def __init__(self, urdf_file, fixed_link=None) -> None:
        self.linkName2IdDic = {}
        self.jointName2IdDic = {}
        self.jointNum = 0
        self.jointInfo = None
        self.robotId = None
        self.urdf = urdf_file
        self.loadRobot(fixed_link)

    def loadRobot(self, fixed_link):
        self.robotId = p.loadURDF(
            self.urdf,
            useFixedBase=(fixed_link == None),
            flags=p.URDF_MERGE_FIXED_LINKS,
        )
        for i in range(p.getNumJoints(self.robotId)):
            self.jointInfo = p.getJointInfo(self.robotId, i)
            print(self.jointInfo)
            self.linkName2IdDic[self.jointInfo[12].decode()] = self.jointInfo[0]
            if self.jointInfo[2] != p.JOINT_FIXED:
                self.jointNum += 1
                self.jointName2IdDic[self.jointInfo[1].decode()] = self.jointInfo[0]
        print("fixed link: ", fixed_link)
        if fixed_link:
            p.createConstraint(
                parentBodyUniqueId=self.robotId,
                parentLinkIndex=self.linkName2IdDic[fixed_link],  # 固定的link ID
                childBodyUniqueId=-1,  # 与世界坐标系连接
                childLinkIndex=-1,  # 固定到世界
                jointType=p.JOINT_FIXED,  # 固定连接
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],  # 父link的坐标
                childFramePosition=[0, 0, 0],  # 世界原点
            )

    def getLinkTransform(self, parent, child):
        parentTrans = None
        childTrans = None
        if parent == "base":
            parentTrans = p.getBasePositionAndOrientation(self.robotId)
        else:
            parentState = p.getLinkState(self.robotId, self.linkName2IdDic[parent])
            parentTrans = (parentState[4], parentState[5])
        if child == "base":
            childTrans = p.getBasePositionAndOrientation(self.robotId)
        else:
            childState = p.getLinkState(self.robotId, self.linkName2IdDic[child])
            childTrans = (childState[4], childState[5])
        parentInv = p.invertTransform(parentTrans[0], parentTrans[1])
        return p.multiplyTransforms(
            parentInv[0], parentInv[1], childTrans[0], childTrans[1]
        )

    def getJointNameList(self, jointIdList=None):
        if jointIdList == None:
            return list(self.jointName2IdDic.keys())
        else:
            return [list(self.jointName2IdDic.keys())[jIdx] for jIdx in jointIdList]

    def getJointStates(self, jointIdList=None):
        if jointIdList == None:
            jointIdList = self.jointName2IdDic.values()
        jPos = [p.getJointState(self.robotId, jIdx)[0] for jIdx in jointIdList]
        return jPos

    def home(self):
        for jIdx in self.jointName2IdDic.values():
            p.setJointMotorControl2(
                self.robotId, jIdx, p.POSITION_CONTROL, 0.0, force=100
            )

    def move_arm(self, target, jointIdList=None):
        if jointIdList == None:
            jointIdList = list(self.jointName2IdDic.values())
        else:
            if len(target) != len(jointIdList):
                print("Target num does not matched!")
                return
        for i in range(len(target)):
            p.setJointMotorControl2(
                self.robotId,
                jointIdList[i],
                p.POSITION_CONTROL,
                target[i],
                force=100,
            )
        for i, joint_id in enumerate(jointIdList):
            p.resetJointState(self.robotId, joint_id, target[i], targetVelocity=0)


if __name__ == "__main__":
    sim = SimEnv()
    robot = RobotDemo()

    # debug usage, add joint state debug bars==============================
    # joint_index_debug_param_dt = {}
    # for i in range(0, p.getNumJoints(robotId)):
    #     joint_info = p.getJointInfo(robotId, i)
    #     print(joint_info)
    #     if joint_info[2] != p.JOINT_FIXED:
    #         joint_index_debug_param_dt[i] = p.addUserDebugParameter(
    #             paramName=joint_info[1].decode("utf-8"),
    #             rangeMin=joint_info[8],
    #             rangeMax=joint_info[9],
    #             startValue=p.getJointState(robotId, i)[0],
    #         )
    # ======================================================================
    while True:
        # debug usage===========================================================
        # for joint_index, param_id in joint_index_debug_param_dt.items():
        #     p.resetJointState(robotId, joint_index, p.readUserDebugParameter(param_id))
        # debug_draw_pose(p.getLinkState(robot_id, self.ee_index)[0:2])
        # ======================================================================
        robot.home()
        parent = "base"
        child = "link_RightHand_Y"
        trans = robot.getLinkTransform(parent, child)
        # print(f"trans from {parent} to {child} : {trans}")
        parent = "base"
        child = "link_RightInHandMiddle_Y"
        trans = robot.getLinkTransform(parent, child)
        # print(f"trans from {parent} to {child} : {trans}")
        # p.getLinkState(robotId, linkId["link_RightInHandMiddle_Y"])
        # rHandState = p.getLinkState(robotId, linkId["link_RightHand_Y"])
        time.sleep(1.0)
