# io_mocap
io数据相关工具

## visulization launch
用于io数据数据可视化
```bash
source install/setup.bash
```

io动捕数据可视化
```bash
ros2 launch io_mocap human_vis.launch.py
```
手部外骨骼数据可视化
```bash
ros2 launch io_mocap exoskeleton_vis.launch.py
```

## xsens to io mocap data adapt(offline)
xsens数据转化为io数据，并将转化后数据以/io_fusion/tf对外发布
```bash
python3 <your_package_dir>/src/io_dev_tools_ros2/io_mocap/script/xsens_io_adapter_offline.py <your_xsen_file_path>
```
## pico tracker to io mocap data adapt(online)
```bash
python3 src/io_dev_tools_ros2/io_mocap/script/pico_io_adapter_online.py
```