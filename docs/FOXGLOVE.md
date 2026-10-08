# Foxglove 操作流程

使用当前 Foxglove 数据源连接扩展电脑的 8765 端口。
截图中的旧相机话题不存在；删除或修改旧相机面板，不沿用 `/robot0/camera/image_raw`。

## 显示

| 面板 | 设置 |
|---|---|
| 3D | 固定参考系 `map`，显示参考系 `map`，先关闭跟随机器人 |
| 网格上轴 | `Z 向上`，与 ROS 的 XYZ 坐标约定一致 |
| 实时点云 | 开启 `/point_cloud2`，点大小 2–3 px |
| GICP 配准点云 | 开启 `/registered_cloud`，坐标已在 `map`，用于观察已通过配准的三维扫描 |
| 三维地图 | 开启 `/map_cloud`，与实时点云使用不同颜色 |
| 图像 | `/camera/image/compressed` |
| 原始消息 | `/localization/status`、`/localization/valid`、`/operator/status` |
| 导航路径 | `/plan`、`/local_plan`，仅导航模式 |
| 二维栅格 | `/map`，按需开启透明度，避免遮住三维点云 |
| 障碍代价地图 | `/local_costmap/costmap`、`/global_costmap/costmap`，按需开启 |

观察稳定的三维场景时，先只开启 `/registered_cloud` 和 `/map_cloud`。
这两个话题保留 XYZ，颜色字段选 `z`（高度）；它们没有 `intensity` 字段。
`/registered_cloud` 使用通过 GICP 检查后的位姿（定位时包含校正滤波），与发布的 TF 一致，只在配准通过时更新；`/map_cloud` 是保存到 PCD 的地图。
地图点云衰减时间保持 0，避免把每次重复发布的全图叠加。
实时 `/point_cloud2` 是机身坐标系单帧扫描，衰减时间 0 时显示每帧采样变化；
在稳定 `odom`/`map` 参考系下可以设置 0.5–1 秒观察扫描覆盖，但显示累积不等同于建图或定位成功。

地图存储分辨率现为 6 厘米，配准仍使用 15 厘米降采样以控制 ARM CPU 消耗。
即使机器人静止，也会每隔 2 秒加入通过匹配检查的扫描以补充地图覆盖；数据预算仍为 20 万地图点。
可选 `dense` 配置使用 2 厘米地图、100 万点预算及 0.5 秒时间戳里程计对齐的扫描窗口，
仍用 15 厘米点云进行 GICP。地图每 3 秒更新一次，点大小可设为 1–2 px，背景选黑色。
这会积累真实雷达回波，点数及高度范围仍由实际扫描覆盖决定；不会生成缺失的墙体或天花板。
静止时超过 5 厘米/约 2° 的单次校正会被拒绝；定位无效时先检查匹配与地图，不能靠放宽阈值继续运动。
传感器/配准不稳定时先运行只读诊断，并让机器狗静止。
诊断中的 `z_span` 检查实际高度范围，TF variation 分别显示里程计与 GICP 校正的位姿变化；
另外列出 `/tf` 发布节点供排查重复节点；这份列表不表示各节点发布了哪一条 TF 边。
诊断也读取原生 `cloud_deskewed`、`voxel_map`
和 `uslam` 点云/里程计；话题存在不等于有数据，也不能直接假定其坐标系与本地图一致。
截图无法确定抖动原因，不应靠延长衰减时间掩盖位姿跳动。

定位刚启动但尚未设置初始位姿时，`map → odom` 不存在；可暂时选 `odom` 查看实时点云，
地图仍在 `map` 下。设置位姿并匹配通过后切回 `map`。不要添加固定的 `map → odom` 来掩盖问题。

相机使用官方 GetImageSample，最高约 2 Hz。只有日志出现
`First camera sample published` 且图像实际刷新才算接通。RPC 超时或非零状态码需要检查固件接口，
不能靠改话题名解决。

点云或三维地图未显示时，在第二个 SSH 终端运行：

```bash
cd ~/go2_nav
git pull --ff-only
bash go2_3d/tools.sh diagnose
```

这会在当前容器中只读采样 12 秒，输出原始与桥接消息数、frame、时间戳、有效 XYZ、
地图数据及两段 TF。文件通过目录挂载生效，不需要重建镜像或重启建图。
Foxglove 订阅日志仅证明订阅建立，不证明收到数据。可暂时将固定/显示参考系都设成
`base_link`，展开“主题”并开启 `/point_cloud2` 检查数据接收；建图成功后将固定/显示参考系都切回
`map` 并开启 `/registered_cloud`、`/map_cloud`，让显示保持在世界坐标系中。

## 建图与保存

SSH 启动 `bash go2_3d/run.sh mapping` 后，静置等待 `/map_cloud`，再用遥控器缓慢走动。
查看地图是否连续且墙体对齐；本模式不发送运动命令。
室内稠密建图可改用 `bash go2_3d/run.sh mapping dense`。重启建图会开始新地图，先保存需要保留的当前地图。

在 Foxglove 添加 **发布（Publish）面板**：

| 用途 | 话题 | 消息类型 | JSON 内容 |
|---|---|---|---|
| 保存地图 | `/mapping/save` | `std_msgs/msg/Empty` | `{}` |
| 启用运动 | `/control/enable` | `std_msgs/msg/Bool` | `{"data":true}` |
| 停止运动 | `/control/enable` | `std_msgs/msg/Bool` | `{"data":false}` |
| 取消导航 | `/navigation/cancel` | `std_msgs/msg/Empty` | `{}` |

不同 Foxglove 版本可能把类型显示为 `std_msgs/Empty` 等，选择服务器实际公布的类型即可。
设置为手动单次发布，避免自动重复保存或重复启用。

保存时点击发布，检查 `/operator/status` 中出现 `Save success=True`，并包含保存目录。
成功意味着同一个目录同时包含 `map.pcd`、`nav.pgm`、`nav.yaml`、`metadata.yaml` 和 `observed_free.npz`。
失败时保留诊断，不把部分生成文件用作导航地图。

## 初始化定位/重定位

保存后通过 SSH 结束建图并运行 `bash go2_3d/run.sh navigation`。
此时加载 `maps/latest` 的三维 PCD 与对应栅格，运动默认关闭。
使用稠密扫描窗口时执行 `bash go2_3d/run.sh navigation dense`；不会改变已经保存的 PCD 分辨率。

在 3D 面板的齿轮设置中，向下找到“发布 / Publish”分组（不是“主题”中的 `/initialpose`）：

1. 类型选择“位姿估计 / Pose estimate”，话题填写 `/initialpose`。
2. 显示参考系设为 `map`，用右侧 `3D` 按钮切换到俯视视图，方便选取 XY 位置。
3. 场景右侧“尺子”下方、带小三角的按钮是发布工具；默认图标是圆靶心。
   长按该按钮约 1 秒可打开菜单，选择 **Publish pose estimate**。
4. 选择类型后，再单击发布工具按钮启用它，按钮应变蓝。
5. 在地图中**单击一次实际位置**，松开鼠标并移动到朝向方向；再**单击一次**确认并发布。
   这里是两次单击，不是 RViz 的按住拖动后松开。每次操作结束后需再次启用工具。

左侧“主题”列表展开 `/initialpose` 只设置位姿消息的显示样式，不会启用鼠标发布工具。
该地图的原点是建图第一帧机身位置；相同站立高度下机身 `z` 大致为 0。
如果当前版本无法选用鼠标工具，使用下方 SSH 方法或 Publish 面板填写真实位置与朝向。
`/initialpose` 必须有 `header.frame_id: map` 和有效的四元数，默认空参考系消息会被 GICP 拒绝。

可直接导入 [预设导航布局](foxglove_go2_navigation.json)。它包含两个俯视 3D 面板：
左面板预设 `/initialpose` 位姿估计，右面板预设 `/goal_pose` 位姿目标；相机、定位状态和
启用/停止/取消按钮同时显示。通过 Foxglove 布局菜单选择“从文件导入 / Import from file”。
Windows PowerShell 下载文件：

```powershell
Invoke-WebRequest -Uri 'https://raw.githubusercontent.com/shan504/go2/main/docs/foxglove_go2_navigation.json' -OutFile "$env:USERPROFILE\Downloads\go2_navigation.json"
```

导入后连接当前 `ws://192.168.123.18:8765` 数据源；导入本身不会发布消息或启用运动。
鼠标工具仍需单击启用，再依次点击位置和朝向。布局只减少设置步骤；桌面版本兼容性待用户导入确认。
布局缩小坐标轴并关闭重叠标签以改善可读性，这不会改变 ROS TF 或修复定位误差。

### 只剩发布按钮、3D 面板不见了

重新导入上面的完整布局，恢复两个 3D 面板和正常分割比例。现有布局中的代价地图
透明度调整不需要拖动面板分割线；发布按钮区域占满窗口时，先恢复布局再查看数据。
也可在 Windows 本机 PowerShell 从机器狗电脑下载当前仓库中的布局：

```powershell
scp unitree@192.168.123.18:/home/unitree/go2_nav/docs/foxglove_go2_navigation.json "$env:USERPROFILE\Downloads\go2_navigation.json"
```

在 Foxglove 顶部布局菜单选择从文件导入，再连接 `ws://192.168.123.18:8765`。
导入只恢复显示和工具设置，不发布初始位姿。若日志最后出现 `user interrupted with ctrl-c`，
先在第一个 SSH 终端重新运行 `bash ~/go2_nav/go2_3d/run.sh navigation dense` 并保持运行，
再从第二个 SSH 终端或 3D 面板设置当前真实初始位姿。
`Invalid frame ID "map"` 是 TF 不可用，需要检查 GICP 是否收到初始位姿并通过匹配；
仅加载地图或导入布局不会建立 `map → odom`。

初始位姿示例（只有回到建图起点、同一朝向时才适用）：

```json
{
  "header":{"stamp":{"sec":0,"nanosec":0},"frame_id":"map"},
  "pose":{
    "pose":{"position":{"x":0,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}},
    "covariance":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  }
}
```

也可通过第二个 SSH 终端初始化。先保持机器人静止，运动关闭；**以下零位姿仅适用于
回到建图起点、同一朝向及站立高度的情况**：

```bash
bash ~/go2_nav/go2_3d/tools.sh shell
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
ros2 topic echo /localization/valid
```

最后一条会持续显示定位有效性，按 Ctrl+C 结束观察，输入 `exit` 返回宿主机。
初始位姿是近似提示；日志收到 seed 不代表已经定位成功，必须等 GICP 匹配通过。
若机器人在其他位置，应填写其真实地图位置及朝向，不能照用零位姿。
**每次重新运行 `run.sh navigation dense` 都需要重新设置初始位姿**，保存的地图不会自动
恢复机器狗当前位姿。启动终端要保持运行，在第二个 SSH 终端初始化和检查。
也可直接运行 `bash go2_3d/run.sh navigation dense X Y YAW_DEGREES [Z]`，
让程序在收到新鲜点云和里程计后发布一次这个实际位姿提示；仍须通过 GICP。
零位姿只用于实际回到建图起点、同朝向和高度。程序不会猜测机器狗已回到起点。
Nav2 保持未激活，直到 GICP 与新鲜 TF 就绪；查看 `/navigation/startup_status`。
启动终端按 Ctrl+C 会关闭整个栈并删除 `go2-3d` 容器；此后 `tools.sh` 会报容器不存在。

没有 `map → odom` 时，全局代价地图会等待 `base_link → map`，局部代价地图也不能
在 Foxglove 的 `map` 参考系下显示。先使定位有效，再在容器 shell 中检查：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

都应为 `active`；若定位已有效而代价地图仍不可见，保留生命周期输出并运行只读诊断。
不要发布固定 `map → odom` 绕过 GICP 来消除等待提示。

等待 `/localization/valid` 连续为 `true`，查看 `/localization/status` 的 fitness/RMSE，
并检查当前点云与三维地图是否重合。单独一个 `true` 不保证场景无歧义。
重新初始化同样使用 `/initialpose`；目标取消前不要在运动中重新设定位姿。
定位使用 5 厘米点云进行细 GICP、20 厘米匹配距离及最多 10 厘米 RMSE。
初次/重定位先粗配准再细配准；连续定位从时间戳里程计预测位姿开始细配准。
仅通过匹配和静止跳变检查的 `map → odom` 校正会进行平移/旋转滤波（`correction_alpha=0.25`），
真实 `odom → base_link` 运动不做该滤波。拒绝匹配或失去数据仍使定位无效并触发停止。
`map → odom` 在两次 GICP 之间按最新真实里程计时间、最多 20 Hz 发布最近通过的校正，
校正和里程计都必须仍然新鲜；不会发布未来时间戳。拒绝匹配、重新初始化或校正过期后
停止刷新 TF。点云仍保留采集时间。这避免每次等待 GICP 完成后才更新 TF 导致激光过滤超时。
这能抑制模拟场景中的小幅校正噪声，不能保证任何实机地图都无抖动；持续拒绝时运行只读诊断。

## 导航

确认定位与现场状态后，在启用运动面板单次发布 `{"data":true}`。
查看 `/operator/status` 确认参数设置成功。随后使用 3D 面板的发布位姿工具：

- 目标话题 `/goal_pose`，类型 `geometry_msgs/msg/PoseStamped`，参考系 `map`。
- 在设置的“发布”分组选择“位姿 / Pose”，话题为 `/goal_pose`；或长按场景右侧发布工具选择 **Publish pose**。
- 再单击工具按钮启用它；在已观察到的空地单击一次，移动鼠标指定到达朝向，再单击一次发布。
- 首次选近距离目标，观察 `/plan`、局部障碍和机器人响应。

新增的 `operator_bridge.py` 把 `/goal_pose` 转换为 `/navigate_to_pose` action。
`/operator/status` 应显示 Nav2 接受目标；定位无效、非导航模式或目标参考系错误时会拒绝。
有进行中的目标时先取消，等待 action 完成，再选新目标。

“启用运动”只是允许运动桥执行 Nav2 的速度命令，不会自己产生前进命令。
`GridBased failed to generate a valid path` 或 action status=6 表示导航目标失败，
需要先排查地图和路径。若重启后一直等待 `base_link → map`，先重新初始化 GICP；
Ctrl+C 后的 ROS context invalid 错误属于关闭阶段，不能据此判断运动桥故障。

## 代价地图与规划失败

旧版导航栅格按固定 map 高度切片，并仅保留每个方向最近回波的自由射线，
可能把倾斜地面当障碍、漏掉已观测的远处空地。停止导航后，运行
`bash ~/go2_nav/go2_3d/tools.sh repair-map` 修复已有图：估计起点附近地面，
按距地面的高度投影障碍，并用真实地面端点补充空地。
修复保留原始 PCD 和旧目录，新版导航要求已有地面模型。新建地图保存时自动生成模型。
实时 `/navigation/obstacle_cloud` 使用同一地面模型及点云采集时间的 TF；未知角度不当作无障碍远距离射线清除。
三维 PCD 自身的漂移或重影不属于这个投影修复范围。

局部代价地图是跟随机器狗的 `4 × 4 米` 窗口，分辨率 5 厘米；全局代价地图与
保存的二维地图范围一致。两者膨胀半径为 28 厘米、衰减系数为 12，外形仍为
`0.80 × 0.44 米`，加 1 厘米 padding。颜色表示的渐变代价不全是禁止通行区。
不要同时叠加全局、局部地图判断通道宽度；预设布局右面板默认仅显示全局代价地图，
检查局部时关闭全局、开启局部。预设代价地图透明度为 0.3；已有布局可手动设置。
检查静态栅格时只开启 `/map`：白色为空地，
黑色为障碍，灰色为未知；当前规划禁止穿越未知区域。

在第二个 SSH 终端检查某个地图坐标的目标，例如失败目标 `(0.35, 1.08)`：

```bash
bash ~/go2_nav/go2_3d/tools.sh navcheck 0.35 1.08
```

该命令只订阅状态、TF、地图和激光，不发布目标或运动。它报告定位有效性、地图实际尺寸、
机器人/目标所在单元的代价值，以及已知区域的中心单元连通性。连通性是诊断线索，
不代替 Nav2 的外形和运动约束检查，也不应用目标容差。目标单元被占据或未知时，
先在已观察的空地选择目标；中心单元连通仍规划失败时，继续检查外形是否贴近障碍、
点云高度筛选以及现场通道宽度。降低膨胀范围不会修复静态地图里错误的障碍或未知区域。
Nav2 的 99 表示按机器狗内切半径计算的禁止进入区，100 表示直接障碍；1–98 是渐变代价。
外形宽度 44 厘米加两边各 1 厘米 padding，对应约 23 厘米内切半径。
缩小外圈渐变半径不会缩小这个真实外形约束。诊断会报告目标与最近直接障碍单元的距离，
以及目标附近最新点云的高度，帮助区分障碍贴近目标和地面误入高度筛选。

取消导航发布 `/navigation/cancel`；停止运动发布 `/control/enable` 为 `false`。
定位匹配失败或失去传感器数据时，运动桥会发 StopMove，同时请求取消目标。
软件停止接口需要先在现场验证，遥控器保留在操作人员手中。

首次运行和 mapping/localization/navigation 模式切换通过 SSH 完成；
当前没有把远程重启容器接入 Foxglove。你可将上述面板保存到现有 Foxglove 布局供以后使用。
