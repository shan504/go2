# GICP 前端与 RTAB-Map 相机辅助闭环后端

这个模式保留原有 GICP 配准和建图采样参数，用 Go2 的单目相机辅助识别曾经到过的位置，再用激光点云验证闭环。相机不提供视觉里程计，也不要求新增深度相机。原有纯激光模式继续保留。

**当前没有农场 rosbag、实机相机标定或这次接入后的机器人试验结果。即使代码与合成测试通过，也不等于已经消除了现场 2–3 m 的闭合误差。** 最终测试结果以本次提交说明及实际运行记录为准。

## 原有系统与职责划分

原系统的 `/odom` 来自 Go2 自带里程计。GICP 用它作为先验，将当前局部点云配准到地图；GICP 的匹配位姿以前保存在 mapper 内部，不能把原 `/odom` 直接当成“GICP 里程计”。原有闭环是本仓库实现的几何候选检索、Open3D GICP 验证和 Open3D 位姿图优化，并不是 RTAB-Map。

新模式将未经过全局闭环修正的 GICP 位姿与对应局部扫描交给独立的 `rtabmap_slam/rtabmap` 节点。原几何闭环在该模式下停用，避免两套后端同时改变历史轨迹。

```text
Go2 原生里程计 ──> odom -> base_link
                         │
L1 局部扫描 ─────────> 原 GICP 前端 ──> gicp_odom 中的位姿 + 局部扫描
                         │                         │
                         └── 保留原始局部关键帧    │
                                                   v
Go2 图像 + 实测标定 ──> 校验/去畸变/有界时间关联 ──> RTAB-Map
                                                   │
                          视觉候选 -> 视觉位姿估计 -> ICP 验证
                                                   │
                          闭环约束 -> 位姿图优化 -> MapData
                                                   │
                          后端适配器：重建历史点云/自由区域/保存地图
```

TF 的职责是：原桥接节点继续发布 `odom -> base_link`；GICP 发布其连续前端与原生里程计之间的关系；后端适配器发布 `map -> gicp_odom`。RTAB-Map 的 `publish_tf` 关闭。不能同时保留原 mapper 的 `map -> odom` 全局边，否则会形成重复父节点或互相覆盖。

`/map_cloud`、最终保存的 PCD、导航栅格、地图起始位姿必须来自同一版优化结果。原始局部关键帧与原前端位姿保留在会话中，优化后的图位姿用于重新放置全部历史观测。没有视觉帧或未被 RTAB-Map 采用的扫描不能直接丢失，需沿时间对应的图节点修正历史观测。仅改变 `map -> gicp_odom` 而不重建历史地图，不算完成闭环接入。

## 使用的开源接口

已核对官方源代码中的以下接口：

- RTAB-Map core `0.22.1`：[`df6300e`](https://github.com/introlab/rtabmap/tree/df6300e0ba3e90058f90b09c4d646279366d3516)。
- ROS 2 Humble wrapper `0.22.1-humble`：[`2a31e43`](https://github.com/introlab/rtabmap_ros/tree/2a31e433d2b072795ae90a62abcdd0c00691e04d)。
- 同时检查了 Humble 分支 `4084dca73201f5621820f945f847a0aa7b1de198` 的对应订阅和 `gen_depth` 实现。安装时仍需记录实际 apt 包版本并执行运行时接口检查；分支名称不能替代版本检查。

`CommonDataSubscriberRGB.cpp` 明确支持同时订阅 `rgb/image`、`rgb/camera_info`、`scan_cloud` 和 `odom`，即单目 RGB + 三维扫描 + 外部里程计。`CoreWrapper.cpp` 的 `gen_depth=true` 把真实激光点投影到已标定的相机中，得到稀疏深度，再使用 `Reg/Strategy=2` 的视觉配准与 ICP 串联验证。它不是生成深度的神经网络，也不增加深度硬件。

关键配置含义：

| 配置 | 目的 |
| --- | --- |
| `subscribe_rgb=true`、`subscribe_depth=false`、`subscribe_scan_cloud=true` | 接收单目图像和真实激光扫描 |
| `gen_depth=true` | 用激光提供图像特征的尺度信息 |
| `Reg/Strategy="2"`、`Vis/EstimationType="1"` | 视觉 PnP 估计闭环相对位姿，再由 ICP 检验/细化 |
| `RGBD/LoopClosureIdentityGuess="false"` | 视觉估计失败时不通过单位初值偷偷退化为 ICP-only 闭环 |
| `RGBD/ProximityBySpace="false"`、`RGBD/ProximityByTime="false"` | 本模式的闭环约束来自视觉候选，不混入独立几何近邻闭环 |
| `RGBD/NeighborLinkRefining="false"` | 不让后端重新估计 GICP 的相邻里程计约束 |
| `publish_tf=false` | 全局 TF 与地图更新由同一个适配器提交 |
| `Mem/RehearsalSimilarity="1.0"`、`Mem/ReduceGraph="false"` | 保持关键帧关联关系，避免后台合并导致帧丢失 |

RTAB-Map 的 ROS 参数使用 ROS 原生布尔值/整数；带 `/` 的核心参数使用字符串。版本不支持的参数不得当作已经生效。特别是所检查的 `rtabmap_slam` RGB 订阅器没有 `approx_sync_max_interval` 参数，不能靠给它填一个不存在的同步参数来保证时间一致。

RTAB-Map 还会尝试使用 `Odometry.header.frame_id -> base_link` 的 TF 覆盖收到的位姿。因此 GICP 输出必须使用独立的 `gicp_odom` 坐标系，不能把 GICP 消息错误标为原生 `odom`，否则实际可能仍然使用原生里程计。

## 启用前必须具备的标定与时间条件

目前 Go2 视频采样是图像流接入，不自动证明图像已校正或与激光硬同步。下面的数据必须来自这台机器及实际使用的分辨率：

1. 相机宽、高、内参 `K`、畸变模型与系数 `D`，以及所用校正矩阵/投影矩阵。可向厂家取得该机标定，或用棋盘格/ChArUco 多角度覆盖整个视场重新标定。广角相机须确认 `equidistant` 与普通 `plumb_bob` 模型，不可混用；改变裁剪/缩放后必须同步变换内参。
2. `base_link -> camera optical frame` 的实测外参。光学坐标通常为 x 向右、y 向下、z 向前；机身坐标通常为 x 向前、y 向左、z 向上。URDF 中用于外观的近似安装位置不能当作激光投影标定。用标定板或固定结构做相机—激光联合标定，并检查投影边缘是否重合。
3. 图像时间的来源、视频传输/解码延迟与抖动。SDK 返回帧后打的时间是接收时间，不是曝光时间。需要传感器采集时间或实测的时间偏移及稳定性；不能凭经验填一个延迟。原始时间戳与关联误差都应记录，超出允许范围时跳过相机约束并报告原因。
4. 图像无严重运动模糊、过曝、遮挡；起点与回到起点有足够相同视角与稳定结构。单目前视相机反方向回来时可能看不到同一场景；不保证每次都有视觉闭环。激光投影到相机视场内的有效深度/特征不足时应拒绝约束，不能放宽到把错误闭环算成功。

禁止填虚构的焦距、零畸变、单位外参来让启动检查通过。已有 PCD 没有逐帧图像、局部扫描和轨迹时，不能仅靠这次新后端回头修正旧地图。

标定文件采用标准 ROS 相机 YAML 字段，并要求额外提供 `calibration_source`、`extrinsics` 和 `timing`。完整字段说明见 [`camera_calibration.py`](../go2_3d/camera_calibration.py) 开头的格式说明。当前适配器支持 `plumb_bob`、`rational_polynomial` 和 `equidistant`；单目校正旋转 `R` 必须为单位阵，四元数必须描述机身到实际光学坐标的旋转。已有预览话题使用接收时间，不能把它标成 `capture_header` 来绕过时延测量。

## 启动、保存与回退

先保留现有 `go2-3d:edu` 镜像，再构建带后端依赖的独立镜像；不需要替换原来的 GICP/Nav2 镜像：

```bash
cd ~/go2_nav
sudo docker build -f Dockerfile.rtabmap -t go2-3d:rtabmap .
```

取得这台机器的真实标定后，将下例路径替换为实际 YAML 路径。`farm_camera` 是示例新地图名，不是修复所依赖的固定地图：

```bash
GO2_LOOP_BACKEND=rtabmap \
GO2_CAMERA_CALIBRATION=/home/unitree/go2_camera_calibrated.yaml \
bash go2_3d/run.sh mapping dense farm_camera
```

启动前会检查标定、光学坐标系是否重复，以及已安装节点的参数服务是否真正支持所需接口；检查失败不会先停止当前运行的容器。每次新建 `maps/rtabmap_sessions/<会话>/`，保存独立数据库、原始关键帧记录和 ROS 日志，不覆盖已有数据库。容器结束前保留完整 stdout 也便于分析：

```bash
sudo docker logs go2-3d > runtime/rtabmap-session.log 2>&1
```

建图后沿用现有保存入口：

```bash
bash go2_3d/tools.sh save
```

新模式由后端地图节点响应保存服务，GICP 前端禁止导出未经全局修正的地图。保存成功只证明该版地图导出完成，**不证明检测到了闭环**；必须同时看后端的闭环约束、优化和地图重建记录。

使用已保存的优化地图时，仍走现有 GICP 定位和 Nav2，不需要让 RTAB-Map 在导航期间再次改图：

```bash
bash go2_3d/tools.sh use-map farm_camera
bash go2_3d/run.sh navigation dense
```

以上环境变量只对那一条建图命令有效；若此前用 `export` 设置过，则先 `unset GO2_LOOP_BACKEND GO2_CAMERA_CALIBRATION`。省略初始坐标时沿用地图自己的保存起点和朝向。回退到原纯激光建图可显式使用：

```bash
GO2_LOOP_BACKEND=lidar bash go2_3d/run.sh mapping dense farm_lidar
```

## 记录与验收

没有可复用的农场 rosbag 时，先记录同一趟“出发—绕行—回到起点”的输入，再用同一份数据做 A/B。至少保留：

- `/point_cloud2`、`/odom` 和原始 `/utlidar/cloud_base`、`/utlidar/robot_odom`（可用时）；
- `/camera/image/compressed`、实际使用的相机内参/原始时间来源；
- `/tf`、`/tf_static`，并明确每条边的发布者；
- 新模式的 GICP 位姿、局部点云、关键帧记录和相机同步诊断；
- RTAB-Map `info`、`mapData`、后端地图更新诊断和完整节点日志。

保存相机标定文件、全部配置、提交号和 RTAB-Map 包版本，与 rosbag 一同归档。只录最终 `/map_cloud` 不能重跑闭环。录制原始输入的 bag 可以包含 TF，但离线重放时必须过滤当次运行生成的 `map`/`gicp_odom` 全局边，避免与本次算法发布的 TF 重复。不要同时连接实机 DDS 与回放源。

在建图运行时，另开终端记录输入与闭环证据；如容器尚无 `ros2 bag`，安装 `ros-humble-rosbag2` 后重建镜像。此命令不发送运动指令，按 Ctrl+C 结束录制：

```bash
cd ~/go2_nav
mkdir -p maps/recordings
sudo docker exec -it go2-3d bash -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 bag record -o "/maps/recordings/farm-loop-$(date -u +%Y%m%dT%H%M%SZ)" \
  /point_cloud2 /odom /camera/image/compressed /tf /tf_static \
  /gicp/odom /gicp/scan_cloud /gicp/keyframe/odom /gicp/keyframe/cloud \
  /camera/loop/image_rect /camera/loop/camera_info /mapping/camera_status \
  /rtabmap/input/image /rtabmap/input/camera_info \
  /rtabmap/input/scan_cloud /rtabmap/input/odom \
  /rtabmap/info /rtabmap/mapData /mapping/loop_closure /rosout
'
```

先用 `ros2 bag info` 确认关键话题有消息。录入大量点云和图像会增加磁盘/网络负载，原始输入与会话关键帧都在机器人本地保存，避免通过手机热点向电脑传输完整地图后才录制。

逐阶段检查下面的证据，不能只看轨迹首尾是否接近：

| 阶段 | 可核查证据 |
| --- | --- |
| 候选检索 | `Info.stats_keys/stats_values` 中的 `Loop/Highest_hypothesis_id/`、`Loop/Highest_hypothesis_value/`；最高候选不等于已接受 |
| 几何验证 | 视觉匹配/内点统计与 `Rejected loop closure A -> B: ...` 日志；在本配置下被接受的全局约束经过视觉 + ICP |
| 约束加入 | `MapData.graph.links` 中 `type=1` 的全局闭环边，记录端点 ID；`Info.loop_closure_id` 辅助核对 |
| 优化执行 | `Loop/Optimization_iterations/`、`Loop/Optimization_error/`、`Timing/Map_optimization/ms` 与图位姿变化 |
| 历史地图更新 | 后端地图版本、重建观测数量、实际输出点云与保存元数据；地图更新必须晚于对应图优化 |

`MapData.nodes[].id/stamp/pose` 提供节点编号、时间和未经优化的里程计位姿；`graph.poses_id/poses` 是优化后的位姿。节点数据可能增量发布，需要缓存 ID 与时间的对应关系。`Info` 中所查版本没有每次全局闭环专用的 ICP RMSE 字段，不能编造一个值；保留拒绝原因及原始日志，必要时通过 RTAB-Map 调试日志或数据库做进一步分析。

默认 `Mem/STMSize=10` 只把旧节点从短期记忆移入工作记忆，不是删除历史图；本配置把时间/内存转存阈值设为零，并禁用节点合并。`Rtabmap/PublishStats=true` 保证持续输出图及统计。GICP 当前没有输出经过标定的位姿协方差，后端采用 RTAB-Map 的默认里程计方差权重；这不是对该机器精度的测量结果，现场验证时应记录并评估该假设。

## 同一份数据离线 A/B

`rtabmap_replay.launch.py` 仅启动数据回放所需节点；不启动 DDS、实时相机、运动控制和 Nav2。`replay_odom_tf.py` 从录制的 `/odom` 重建唯一的 `odom -> base_link`。所有回放节点使用 `/clock`，不要回放原始 bag 中的 `/tf` 或 `/tf_static` 来覆盖新计算的坐标系。

以下两轮分别将 `backend` 设置为 `lidar`、`rtabmap`。把 bag 和标定路径替换为实测文件，每轮结束后关闭该回放容器，再启动下一轮；`trial_dir` 每次自动新建，不复用数据库。

```bash
cd ~/go2_nav
backend=lidar
bag_dir=/home/unitree/bags/farm_loop
calibration_file=/home/unitree/go2_camera_calibrated.yaml
trial_dir="$PWD/runtime/loop_ab/${backend}-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$trial_dir"
sudo docker run --rm -it --name go2-loop-replay --network none \
  -e ROS_DOMAIN_ID=87 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  -v "$PWD:/opt/go2_project:ro" -v "$trial_dir:/results:rw" \
  -v "$bag_dir:/bag:ro" -v "$calibration_file:/calibration/camera.yaml:ro" \
  --entrypoint bash go2-3d:rtabmap -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec ros2 launch /opt/go2_project/go2_3d/rtabmap_replay.launch.py \
  loop_backend:="$1" map_name:="ab_$1" output_directory:=/results/export \
  session_directory:=/results/session camera_calibration:=/calibration/camera.yaml
' bash "$backend"
```

另开终端，仅播放三类输入；两轮使用相同播放速率。先用低速离线回放检查漏帧，不能用它代替 Jetson 实时性能测试。

```bash
sudo docker exec -it go2-loop-replay bash -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 bag play /bag --clock --rate 0.5 \
  --topics /odom /point_cloud2 /camera/image/compressed
'
```

回放完成后确认后端已经消费完图节点、重建完成，再保存并导出日志；若保存服务报告处理中，继续检查后端状态，不能拿尚未应用优化的地图当作结果。

```bash
sudo docker exec -it go2-loop-replay bash -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 service call /save_3d_map std_srvs/srv/Trigger "{}"
'
sudo docker logs go2-loop-replay > "runtime/loop_ab/replay-$(date -u +%Y%m%dT%H%M%SZ).log" 2>&1
```

分别归档两轮日志，保存结果位于各自 `trial_dir/export`；RTAB-Map 数据库及关键帧位于 `trial_dir/session`。这套入口使同一原始输入可以重复比较，但当前没有真实农场 bag，尚未执行现场数据的 A/B。

同一 bag 分别运行原纯激光闭环与新相机后端，配置除后端开关外保持一致，分别使用新会话数据库。记录：

- **闭合误差**：人工确认回到同一物理位置和朝向的两帧，分别计算优化前后平移距离与旋转角。不要拿经过闭环约束强制连接的两个任意帧直接当作地面真值；单凭 SLAM 自身闭合只能衡量内部一致性。
- **固定结构接缝**：在起点选同一树干、路缘或固定标志，分开取出发与返回时段的扫描，在同一坐标系下比较表面间距、重影宽度和局部重叠。草叶、行人、摆动树冠不能作为主要参考。
- **地图一致性**：显示地图与保存 PCD/栅格相符；关闭重启并重新选择该地图后，定位与导航使用的是同一份优化地图和变换后的起始位姿。
- **资源与稳定性**：记录回环/重建耗时、峰值内存、相机丢帧/时间关联拒绝数；在 Jetson 上实测，不能把云端 x86 测试耗时当成机器人性能。

数据不支持视觉候选或几何验证失败时，记录“未闭环”并保留可回退的原始会话，不宣称误差已经消除。

## 本次验证范围（2026-11-12）

以下在云端 x86_64 环境验证，合成标定只用于测试，没有写入机器人配置。

| 验证 | 结果与边界 |
| --- | --- |
| 相机、时间关联、图修正、地图导出、报告工具及既有单元测试 | **141 项通过**；涵盖缺失标定拒绝、不同地图原点、非均匀轨迹修正、尾部未优化时阻止保存、原地图版本保留 |
| `rtabmap_camera_smoke.py` | 真实 ROS2 Humble 图像/CameraInfo/静态 TF 发布；重复、过期、尺寸错误等输入拒绝 |
| `rtabmap_frontend_smoke.py` | 真实 GICP 输出与原生里程计区分、局部扫描时间对应、后端就绪前不提交首帧、旧闭环停用、前端禁止保存未优化地图 |
| `rtabmap_backend_smoke.py` | 用合成优化图验证完整点云与自由空间重建、PCD/栅格/起点一致、缺图恢复、保存门控、异步导出和正常 SIGINT 退出；这不是 RTAB-Map 实测闭环 |
| `rtabmap_launch_smoke.py` | 6 项 ROS launch 检查；标准/稠密参数、模式开关、字符串参数、模拟时钟和旧数据库保护 |
| `rtabmap_ros_smoke.py` | **实际 RTAB-Map core/ROS 0.23.13**，生产配置下 6 帧合成输入产生 6 条 Info、5 个图节点、4 条相邻边、3298 个有效激光投影视觉特征深度；原始位姿关联正确，RTAB 没有发布全局 TF。未将相邻边称为闭环边 |
| 原流程回归 | `ros_smoke.py`、`mapping_loop_smoke.py`、`map_start_localization_smoke.py` 通过，覆盖原建图/定位、原几何闭环与保存、不同地图的起点及启动门控 |

实际 RTAB 接口测试使用官方 `introlab3it/rtabmap_ros:humble` 的 amd64 manifest
`sha256:ecd9f7309e2e66c015a7574faafd5db0c08321f6d3409d91e2f154611b65d91d`；
独立参数预检 `check_rtabmap.probe_backend()` 同样通过。Go2 的 ARM SDK 扩展镜像尚未在该机器构建验证，Jetson 实时负载、真实标定、农场闭环与同一现场 bag 的 A/B 均未验证。

在具备依赖的测试环境中，可运行单元测试：

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```

上述 ROS 脚本分别通过 `python3 tests/<脚本名>` 执行。使用独立无网络测试容器，或与机器人隔离的 ROS domain；它们使用合成传感器及测试图，不应混入正在运行的实机话题。

## 会话轨迹与接缝报告

停止录制和建图后，可以离线读取某次会话的原始关键帧、GICP 位姿和 `graph_latest.json`：

```bash
cd ~/go2_nav
session_dir=/home/unitree/go2_nav/maps/rtabmap_sessions/实际会话目录
sudo docker run --rm --network none \
  -v "$PWD:/opt/go2_project:ro" -v "$session_dir:/session:ro" \
  --entrypoint python3 go2-3d:rtabmap \
  /opt/go2_project/go2_3d/rtabmap_session_report.py /session \
  --first-index 0 --last-index -1
```

索引按已保存关键帧的时间排序，`-1` 表示最后一帧。人工确认是哪两帧回到同一物理位置后，再选择对应索引；默认首尾不一定是同一位置。`--same-place-confirmed` 仅记录操作员确认，不会把内部闭合误差当成独立地面真值。

报告列出原始 GICP 与优化后首尾的三维距离、水平距离、旋转角，接受的全局闭环边、图提交记录和最近的候选/地图更新事件。若没有 `type=1` 的全局闭环边，即使首尾距离为零，也明确输出 `NOT_VERIFIED_no_accepted_global_loop_constraint`。未被图节点覆盖的首尾或遗漏的历史锚点单独标识。

接缝比较可另加 `--roi XMIN YMIN ZMIN XMAX YMAX ZMAX`，范围必须由操作者在地图中选择同一固定结构，并同时覆盖优化前后的结构位置。工具对这两帧的同一世界坐标 ROI 计算双向最近邻距离中位数、95 分位数，以及阈值内比例；不会重新做 ICP 拟合来压低误差。默认比较阈值为 0.20 m，可用 `--distance-threshold` 改变，**它只是覆盖率统计阈值，不是宣称的定位精度**。ROI 内不足 20 个点会报告数据不足，而不是“接缝误差为零”。该报告仍需配合可视化检查和独立位置测量判断现场效果。
