# io_robot
io数据相关工具

## robot model launch
用于机器人仿真与可视化
```
source install/setup.bash
```

io动捕数据可视化
```
ros2 launch io_robot robot_model.launch.py robot_path:=<your_robot_path> robot_name:=<your_robot_name> use_sim:=True use_gui:=False
```
参数解释
```
robot_path:=/home/io/io_teleop_robot_descriptions/  #机器人信息路径（urdf和mesh路径）
robot_name:=DualPiper                               #机器人名字（与config中文件夹名字保持一致）
use_sim:=True                                       #使用实际物理仿真器 default=True
use_gui:=True                                       #使用joint_state_publisher_gui default=True
```
## 文件结构
```
├── CMakeLists.txt
├── README.md
├── config
│   ├── DualPiper
│   │   ├── rviz.rviz
│   │   └── sim_ros.yml
│   └── G1
│       ├── rviz.rviz
│       └── sim_ros.yml
├── launch
│   └── robot_model.launch.py
├── package.xml
└── src
    ├── __init__.py
    ├── robot_sim_bullet.py
    └── robot_sim_ros2.py
```
## 新增机器人配置
在config中新增文件夹，名字为期望机器人名字，新增rviz与sim_ros.yml文件

yml文件配置
```
urdf: "description/urdf/G1.urdf"            #urdf的具体路径
sub_topic: /joint_cmd                       #接受指令joint command topic
pub_topic: /joint_states                    #发布状态Joint states topic
# init_q:                                   #期望起始（只填写非零期望位置，其他关节默认为0）
#   right_shoulder_pitch_joint: 1.3
#   right_shoulder_roll_joint: 0.2
#   right_shoulder_yaw_joint: 0.5
#   right_elbow_joint: 0.1
#   right_wrist_roll_joint: -0.6
# fixed_link: "torso_link"                  #期望锁定link（默认会锁定urdf基link）
```