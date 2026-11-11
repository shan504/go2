# 1000万点容量配置与验证

标准、稠密及不传配置文件的 Mapper 默认 `max_map_points` 统一为 10,000,000。
这个兼容名称表示一次性容量告警阈值，现有建图存储本来就没有300万硬截断。
超过告警阈值仍保留合格扫描和旧区域，不裁图、不改变2厘米/6厘米保留分辨率，
配准仍使用原来的局部粗地图和接受门限。

共享启动文件将 `send_buffer_limit` 默认值从33,554,432改为268,435,456（256 MiB），
并显式传给 Foxglove 子启动文件。所有地图和 mapping/localization/navigation 入口共用。
用户显式传入的启动参数仍可覆盖默认值。

1000万 XYZ float32 点需要120,000,000字节（约114.44 MiB），实际ROS CDR消息为
120,000,109字节。256 MiB能容纳两帧这种消息及一部分协议/其他话题开销，
仅作为旧版按字节限制发送缓冲的桥的容量预算；这不是网络吞吐保证。

## Foxglove版本差异

现场出现的 `foxglove::websocket::connected_client::send_lossy` 日志属于新版Rust队列路径。
检查官方 foxglove-sdk `8dc7525eda8c9b8cbe1b64cc885a14452db832b5` 后确认，该版ROS2桥
仍声明 `send_buffer_limit`，却没有将它用于实际SDK发送队列；实际队列使用
`message_backlog_size`（默认1024条消息）。因此，回读到32 MiB不能证明现场35.7 MiB
点云因单帧容量被拒绝；此前把这两者直接联系的判断需要纠正。

官方依据：[参数声明](https://github.com/foxglove/foxglove-sdk/blob/8dc7525eda8c9b8cbe1b64cc885a14452db832b5/ros/src/foxglove_bridge/src/param_utils.cpp)、
[ROS2桥实现](https://github.com/foxglove/foxglove-sdk/blob/8dc7525eda8c9b8cbe1b64cc885a14452db832b5/ros/src/foxglove_bridge/src/ros2_foxglove_bridge.cpp)、
[消息队列](https://github.com/foxglove/foxglove-sdk/blob/8dc7525eda8c9b8cbe1b64cc885a14452db832b5/rust/foxglove/src/websocket/connected_client.rs)。

本次保留新版队列长度，避免把慢网络变成更大的内存积压。热点、客户端性能和实际桥版本
仍决定显示表现。本次容量配置不修复现场GICP零匹配；该问题需独立诊断。

## 验证

`tests/point_cloud_capacity_smoke.py` 是显式运行的大容量回归，不加入每次快速单测：

```bash
cd /workspace/go2
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 .venv/bin/python tests/point_cloud_capacity_smoke.py --storage
# ROS2 Humble shell中，项目根目录执行：
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 python3 tests/point_cloud_capacity_smoke.py --ros
```

- 标准6厘米和稠密2厘米各实际构建1000万个唯一细图点，含负坐标及空间块边界；完整快照、
  PCD写入/重读及全量坐标检查通过。追加超过1000万的点后，旧数据、旧快照和新点均保留。
- 真实ROS PointCloud2序列化/反序列化1000万点，完整坐标一致，CDR为120,000,109字节。
  旧32 MiB启动配置的字节预算对照失败；新256 MiB预算通过。这项对照只代表旧桥预算。
- `navigation_launch_smoke.py` 实际求值三个启动模式×两种密度，读取ROS节点参数，验证
  1000万告警、Foxglove include参数传播、显式覆盖及原有几何/定位门限。
- 官方新版 `foxglove-sdk 0.30.3` 的本机WebSocket实测：订阅接收两帧1000万点的真实ROS
  CDR消息，帧长120,000,122字节；接收内容全字节比较及SHA256均一致。客户端为
  `websockets 17.4`。这证明该SDK可以传输这种单帧大小，不代表用户实际ROS桥版本、
  Foxglove客户端渲染或手机热点性能已验证。
- 104项算法单测通过。

云端x86_64存储/PCD测试标准约31.93秒、稠密约30.34秒，整个存储测试峰值RSS约2622 MiB；
ROS消息测试约2.08秒、峰值RSS约860 MiB。本机WebSocket两帧传输含校验分别约2.02秒、
1.28秒，测试进程峰值约725 MiB。这些是测试进程资源用量，不代表Jetson上完整
建图、回环、保存、Foxglove同时运行的内存或速度，不是1000万点实机实时性能验收。

## 更新与当前未保存建图

源码通过挂载生效，不需要为此重建SDK镜像。新默认值在下一次启动进程时读取。
Foxglove的该参数是启动期只读参数，不能用 `ros2 param set` 热更新。

当前建图仍在内存中时，先保留进程。需要结束本次建图时，运行
`bash go2_3d/tools.sh save`，确认返回 `success=True` 后再停止；保存失败时继续保留进程，
不能用重启来替代保存。下一次 `run.sh mapping dense` 会开启新的建图会话，
不会续接上一次进程的内存轨迹。已有保存版本继续按原来的定位/导航入口加载。

```bash
cd /home/unitree/go2_nav
git pull --ff-only
# 完成本次保存、停止后，按用途使用原来的启动命令，例如下一次新建图：
bash go2_3d/run.sh mapping dense
```
