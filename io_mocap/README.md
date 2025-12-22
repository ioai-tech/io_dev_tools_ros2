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
将pico实时人体数据转化为/io_fusion/tf对外发布
```bash
python3 src/io_dev_tools_ros2/io_mocap/script/pico_io_adapter_online.py
```
## from io mocap(ros2bag or mcap) to smplx data(offline)
将离线的含有/io_fusion/tf的ros2bag or mcap数据转成smplx的npz格式
```bash
python src/io_dev_tools_ros2/io_mocap/script/io_smplx_adapter_offline.py \
    --file_path <your_mocap_file> \
    --output_path <smplx_file> \
    --is_old_data True or False \
    --is_pitch_correction True or False
```
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--file_path` | 字符串 | **必需** | 输入文件路径 |
| `--output_path` | 字符串 | None | 输出文件路径 |
| `--is_old_data` | 布尔值 | False | 是否为旧版本数据 |
| `--is_pitch_correction` | 布尔值 | False | 是否启用root的pitch校正 |