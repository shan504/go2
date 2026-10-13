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

检查可通行栅格时，暂时关闭该面板中的 `/map_cloud`、`/registered_cloud`、`/point_cloud2`，
只显示 `/map` 与代价地图。按 z 着色的蓝色点云会遮住白色格子，颜色不是 Nav2 是否可通行的判据。

观察稳定的三维场景时，先只开启 `/registered_cloud` 和 `/map_cloud`。
这两个话题保留 XYZ，颜色字段选 `z`（高度）；它们没有 `intensity` 字段。
`/registered_cloud` 使用通过 GICP 检查后的位姿（定位时包含校正滤波），与发布的 TF 一致，只在配准通过时更新；`/map_cloud` 是保存到 PCD 的地图。
地图点云衰减时间保持 0，避免把每次重复发布的全图叠加。
实时 `/point_cloud2` 是机身坐标系单帧扫描，衰减时间 0 时显示每帧采样变化；
在稳定 `odom`/`map` 参考系下可以设置 0.5–1 秒观察扫描覆盖，但显示累积不等同于建图或定位成功。

地图存储分辨率现为 6 厘米，配准仍使用 15 厘米降采样以控制 ARM CPU 消耗。
即使机器人静止，也会每隔 2 秒加入通过匹配检查的扫描以补充地图覆盖；数据预算仍为 20 万地图点。
可选 `dense` 配置使用 2 厘米地图、100 万点预算及 0.5 秒时间戳里程计对齐的扫描窗口，
建图仍用15厘米点云进行GICP。定位/导航使用 `scan_window=0.5`，累积最近约8帧，
配准降采样为10厘米以控制CPU负载，完整保存地图保留。
地图每 3 秒更新一次，点大小可设为 1–2 px，背景选黑色。
这会积累真实雷达回波，点数及高度范围仍由实际扫描覆盖决定；不会生成缺失的墙体或天花板。
建图静止校正仍限制5厘米/约2°。按用户巡检需求，定位默认允许25厘米/8°的小校正，
单次低质量扫描可沿用仍在原有1秒期限内的已接受结果，失败扫描不会续期。
持续失配到期或超过容限的大跳变仍停止运动；实际参数可由 `tools.sh motioncheck` 读回。
传感器/配准不稳定时先运行只读诊断，并让机器狗静止。
诊断中的 `z_span` 检查实际高度范围，TF variation 分别显示里程计与 GICP 校正的位姿变化；
建图时，连续里程计变化小于2毫米/0.002rad，会保持已建立的 `map→odom` 锚点，
状态显示 `mapping_anchor=held`；新扫描仍通过匹配与锚点位姿质量检查后积累。
走动或转身时显示 `mapping_anchor=updated`，继续正常GICP校正。超限跳变仍拒绝并停止刷新TF。
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
此时从 `maps/latest` 生成完整的 `runtime/navigation_map/` 运行副本：三维PCD与导航栅格
一起刚体对齐地面，地面 `z=0`，栅格默认5厘米。定位、Nav2及实时障碍使用同一坐标。
旧图、新图和只定位模式都自动应用这一公共流程，运动默认关闭；原始文件和历史版本保留。
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
实时建图的原点是第一帧机身位置。保存和加载后的公共地图以地面 `z=0` 为约定，
机身在该图测得的站立高度，无需手动执行 `level-map`。
鼠标工具仍发送二维位姿 `z=0`，定位节点自动补入新地图的机身高度；不需要手填约 0.38 米。
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
上述零位姿同样适用于已经 `level-map` 的地图：程序补入高度，再由 GICP 求真实六自由度机身位姿。
若机器人在其他位置，应填写其真实地图位置及朝向，不能照用零位姿。
**每次重新运行 `run.sh navigation dense` 都需要重新设置初始位姿**，保存的地图不会自动
恢复机器狗当前位姿。启动终端要保持运行，在第二个 SSH 终端初始化和检查。
也可直接运行 `bash go2_3d/run.sh navigation dense X Y YAW_DEGREES [Z]`，
让程序在收到新鲜点云和里程计后发布一次这个实际位姿提示；仍须通过 GICP。
`level-map` 后省略 Z 或设为 0 表示使用保存的站立高度；显式非零 Z 表示机身在新 map 中的高度。
零位姿只用于实际回到建图起点、同朝向和高度。程序不会猜测机器狗已回到起点。
Nav2 保持未激活，直到 GICP 与新鲜 TF 就绪；查看 `/navigation/startup_status`。
启动终端按 Ctrl+C 会关闭整个栈并删除 `go2-3d` 容器；此后 `tools.sh` 会报容器不存在。

没有 `map → odom` 时，全局代价地图会等待 `base_link → map`，局部代价地图也不能
在 Foxglove 的 `map` 参考系下显示。先使定位有效，再在容器 shell 中检查：

```bash
ros2 lifecycle get --no-daemon /controller_server
ros2 lifecycle get --no-daemon /planner_server
```

都应为 `active`；若定位已有效而代价地图仍不可见，保留生命周期输出并运行只读诊断。
`tools.sh navcheck X Y` 也直接查询七个 Nav2 节点的生命周期服务，并读取
`/navigation/startup_status`，不依赖 ROS CLI 后台进程。
`xmlrpc.client.Fault ... !rclpy.ok()` 是 CLI 后台进程错误，不能用它判断 Nav2 状态。
不要发布固定 `map → odom` 绕过 GICP 来消除等待提示。

等待 `/localization/valid` 连续为 `true`，查看 `/localization/status` 的 fitness/RMSE，
并检查当前点云与三维地图是否重合。单独一个 `true` 不保证场景无歧义。
重新初始化同样使用 `/initialpose`；目标取消前不要在运动中重新设定位姿。
巡检定位累积最近0.5秒扫描，使用10厘米配准点云、30厘米匹配距离、最低30%重叠及最多15厘米RMSE。
初次/重定位先粗配准再细配准；连续定位从时间戳里程计预测位姿开始细配准。
仅通过匹配和静止跳变检查的 `map → odom` 校正会进行平移/旋转滤波（`correction_alpha=0.25`），
真实 `odom → base_link` 运动不做该滤波。单次低质量匹配可沿用原期限内的估计；
持续失配/失去数据到期或大跳变使定位无效并触发停止。
`map → odom` 在两次 GICP 之间按最新真实里程计时间、最多 20 Hz 发布最近通过的校正，
校正和里程计都必须仍然新鲜；不会发布未来时间戳。大跳变、重新初始化或校正过期后
停止刷新 TF。点云仍保留采集时间。这避免每次等待 GICP 完成后才更新 TF 导致激光过滤超时。
这能抑制模拟场景中的小幅校正噪声，不能保证任何实机地图都无抖动；持续拒绝时运行只读诊断。

## 导航

使用 3D 面板的发布位姿工具连续选择多个目标。
`/operator/status` 显示 `Goal queued`，新路线先收集，上一轮运动开关仍开启也不会提前出发。
再在启用运动面板单次发布 `{"data":true}`；程序确认桥的实际启用参数、重新检查定位/TF/目标格后，
才发送 Nav2 action。每条新路线都先选点、再启用一次。鼠标操作如下：

- 目标话题 `/goal_pose`，类型 `geometry_msgs/msg/PoseStamped`，参考系 `map`。
- 在设置的“发布”分组选择“位姿 / Pose”，话题为 `/goal_pose`；或长按场景右侧发布工具选择 **Publish pose**。
- 再单击工具按钮启用它；在已观察到的空地单击一次，再移动鼠标、单击一次形成箭头并发布。
  两次点击都在同一个Pose工具内完成，不需要选择Click point；工具若退出激活状态，下一个点前重新启用Pose。
  巡检按位置到达：当前距离目标≤30厘米即可，终点不要求对齐这个箭头。
- 首次选近距离目标，观察 `/plan`、局部障碍和机器人响应。

新增的 `operator_bridge.py` 把 `/goal_pose` 转换为 `/navigate_to_pose` action。
Nav2 原生的 `/goal_pose` 订阅在启动组内重映射，避免绕过操作节点或重复发 action。
每完成一个目标应显示 `Waypoint #1/#2/#3 appended`，`queued`累计增加；点启用后才显示Nav2接受目标。
非导航模式或目标参考系错误时会拒绝。定位无效时可保存静态地图上的目标，但不能启动导航。
原始地图目标值 100、代价地图目标值 99/100、未知目标和过期代价地图也会明确拒绝，
信息包含话题、格子和数值。地图中固定目标归一化为最新时间语义和 yaw 姿态。
规划失败不再自动执行原地 Spin 或 BackUp，避免把恢复动作误认为目标导航。
每个目标追加到同一队列，当前点真实成功后才发送下一点；运行中也可追加，中途不要点停止/取消。
停止或取消会清空路线。定位失效会停车、取消当前action并保留当前点与后续点；
定位恢复后点“启用运动”继续，不会自动恢复。真实规划/控制失败仍需显式“重试暂停路线”。
在主题显示中开启 `/navigation/waypoints` 查看全部编号；`/goal_pose`自身仅渲染最新箭头。

点云和TF分别经DDS到达，点云先到时会暂存并重试其原时间戳对应的TF，不直接丢帧。
仍维持1秒原始数据期限，过期或非法变换不能恢复定位。`/localization/status` 的
`clouds_received/aligned/tf_waits/expired/pending` 区分收包、对齐、等待和丢弃；
`processing_time/source_age` 报告配准耗时和该次结果所用点云的原始接收年龄。
`tools.sh enable/disable` 也通过相同操作节点，支持上述顺序。
Nav2 action 不可用时，启用与目标操作会拒绝并附上最新启动状态；仅定位有效不足以启动导航。
启动状态现在分别报告等待定位、TF/时间戳、持续就绪和生命周期管理服务。

“启用运动”只是允许运动桥执行 Nav2 的速度命令，不会自己产生前进命令。
`GridBased failed to generate a valid path` 表示规划失败。
已收到路径后报 `Failed to make progress` 则需检查速度执行链路；status=6 仅表示 action 已中止，
不能单凭它判断地图有问题。第二个 SSH 终端先运行 `tools.sh motioncheck 30`，
看到 `READY` 后再选择新目标、启用运动；采集覆盖目标开始到失败的整个窗口，
避免在目标中止后读取到零值而误判。输出同时保存到 `runtime/velocity-debug.txt`。
它检查真实端点、消息类型/QoS、平滑器生命周期、`/cmd_vel_nav`、`/cmd_vel`、
Sport 请求/配对响应、桥状态和 `/odom`，不发送任何速度或修改参数。
也可添加“原始消息 / Raw Messages”面板选择 `/control/status`：它显示桥被哪个条件阻断、
Move 1008 请求数/最近速度和本桥对应的 Sport 响应 code。`received zero /cmd_vel` 表示桥接到的是零速度；
`forwarding Move requests` 及新鲜的 `last_Move_age` 表示已发请求，仍需真实运动确认。
没有 Sport 响应不能单独证明失败；响应只按本桥请求 ID 关联，不混入其他 SDK 客户端。
新版桥的 `cmd_received` 是原始接收数，`cmd_gate_accepted` 是进入命令门控数，
`cmd_dropped` 按禁用、定位/传感器失效或非有限速度列出丢弃数。
新字段在导航下次重启后生效，当前容器也可以先运行速度采集。
若已确认非零 `/cmd_vel` 和本桥非零 Move 请求，下一步读取底盘状态：
`tools.sh robotcheck 20`；它只查询运动模式、服务列表和内置避障开关，不改模式。
它也尝试读取 MCF Sport GetState（API1034），不支持时原样报告结果。
需要区分 Nav2 与底盘接口时，按 [README 的手动底盘测试](../README.md#已发-move-但没有前进底盘执行检查)
先 `tools.sh disable` 再 `tools.sh drivecheck`；固定 `0.15 m/s` 两秒后发送 Stop，
Ctrl+C 也发送 Stop，输出保存到 `runtime/drive-debug.txt`。
“停止运动＋取消导航”只控制本程序的导航桥；官方遥控器仍可控制机器人。
独立手动测试过程中用 Ctrl+C 停止，不以 Foxglove 导航桥按钮替代。
直接 Sport 测试已发送成功但只有探身、且原生避障 enable=True 时，可以按 README
运行 `tools.sh drivecheck-native`。它使用官方内置避障 API 输入和速度路径，
保留避障开关，结束会发零速度、Stop 并释放 API 输入；不自动修改导航桥后端或步态。
四足 Go2 若两条接口均只探身，可按 README 执行 `tools.sh drivecheck-balanced`：
显式做一次 BalanceStand1002，等待匹配成功后再运行原速度的两秒 Sport 测试。
这只验证站立准备假设，不自动加入正常导航；记录保存为 `runtime/balanced-drive-debug.txt`。
若重启后一直等待 `base_link → map`，先重新初始化 GICP；
Ctrl+C 后的 ROS context invalid 错误属于关闭阶段，不能据此判断运动桥故障。

## 代价地图与规划失败

旧版导航栅格按固定 map 高度切片，并仅保留每个方向最近回波的自由射线，
可能把倾斜地面当障碍、漏掉已观测的远处空地。现在每次导航启动会自动生成运行栅格：
复用该图地面模型，旧图缺少模型时估计起点附近地面，按当前距地面的高度阈值投影障碍，
并用真实地面端点补充空地。已有自由区域按面积转换；旧调平图从保留祖先恢复缺失的自由面积。
新建地图保存时自动保留原始观测版本，再生成整张PCD与5厘米栅格地面对齐的完整版本。
定位和导航启动时也对任意所选旧图应用相同处理，已对齐地图不会再次抬高或旋转。
`repair-map` 仍可显式保存独立修复版本，启动导航无需先逐张调用该命令。
实时 `/navigation/obstacle_cloud` 使用同一地面模型及点云采集时间的 TF；未知角度不当作无障碍远距离射线清除。
三维 PCD 自身的漂移或重影不属于这个投影修复范围。

全局 Nav2 轮廓使用动态 `base_footprint`，按真实 map 位姿投影到栅格 XY 平面，
不再通过 URDF 固定关节跟随机身 roll/pitch。yaw 保持实际读数，不强行与地图边缘平行。
局部栅格仍使用 odom；如果 map/odom 本身有真实三维倾斜，两张栅格显示仍可能不完全共面。
新建图已改为保留里程计 Z 轴；旧 PCD 不会因此自动旋转或重建。

请导出实际地图供离线检查，而不是反复更改膨胀半径：

```bash
cd ~/go2_nav
bash go2_3d/tools.sh export-map
```

在 Windows PowerShell 下载生成的文件：

```powershell
scp unitree@192.168.123.18:/home/unitree/go2_nav/runtime/go2-map-debug.zip "$env:USERPROFILE\Downloads\go2-map-debug.zip"
```

包中包含当前 `map.pcd`、原始及运行PGM/YAML、地面模型、生成配置和最近容器日志；地图和机器人状态不变。

局部代价地图是跟随机器狗的 `4 × 4 米` 窗口，分辨率 5 厘米；全局代价地图与
保存的二维地图范围一致。两者总膨胀半径现为 5 厘米、衰减系数为 12，外形仍为
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
用户明确要求总膨胀半径为 5 厘米：`prepare.py` 已固定为 `0.05`，不是在内切半径上再加 5 厘米。
当前容器可以保持运行，先停止运动，再运行 `bash ~/go2_nav/go2_3d/tools.sh inflation-5cm`，
直接修改两张地图并回读实际参数。无需重建镜像或地图；下一次重启也保持 0.05。
5 厘米总半径低于机器狗内切半径，Nav2 的对应警告不表示外形检查被禁用。
禁用运动时选目标会暂存，启用确认后才启动 action；进展检查从控制器收到目标开始计时。
全局和局部现均在障碍层后、膨胀层前加入 Nav2 官方 `DenoiseLayer`：
八方向相连、占据格不足 8 格的独立小块会被移除，因此不再膨胀。
阈值见 `go2_3d/navigation_filter.yaml`；修改后重启导航。
已有地图在下次导航启动时自动清理运行栅格，使显示、目标检查和 Nav2 使用同一规则。
`clean-map` 仅用于另存清理版本；新地图保存时也会自动清理。
完整 PCD 仍用于三维定位，`/map_cloud` 中的小物体点云可以继续显示。
这种过滤按格数忽略小块，也可能忽略真实细小物体；八格及以上的连续障碍仍保留。
控制器仍按真实外形检查轨迹。诊断会报告目标与最近直接障碍单元的距离，
以及目标附近最新点云的高度，帮助区分障碍贴近目标和地面误入高度筛选。

取消导航发布 `/navigation/cancel`；停止运动发布 `/control/enable` 为 `false`。
定位匹配失败或失去传感器数据时，运动桥会发 StopMove，同时请求取消目标。
软件停止接口需要先在现场验证，遥控器保留在操作人员手中。

当前实机已确认普通 Sport 在 `0.30m/s` 时正常迈步，两秒内前进约42厘米；
此前原生 Sport、原生避障及 BalanceStand 准备后的 `0.15m/s` 都只让机身前探。
三维导航已统一控制器、平滑器和桥的0.30m/s上限，DWB选择停止/转向或0.30m/s前进，
平滑器完成短暂加减速，地图和5厘米膨胀层不变。

保持狗站稳、当前启动终端运行，在第二个 SSH 终端执行：

```bash
cd ~/go2_nav && git pull --ff-only && bash go2_3d/tools.sh restart-navigation
```

命令禁用运动并清除旧目标，读取当前有效 GICP XYZ/朝向，保留该位置重启导航。
默认重启使用 `dense`，恢复最近0.5秒多帧定位；用10厘米配准降采样降低计算负载。
地图文件不重建、不删减，空包不会刷新旧几何的1秒有效期限。
重启期间不移动狗，也不要重新用 `0 0 0` 初始化已经前进后的位姿。
确认 `/localization/valid=true` 和 Nav2 启动成功后，重新单击前方1～1.5米自由目标和朝向，
再点“启用运动”。保留位姿不会保留旧目标或自动启用；读取失败保留原容器并报错。

直行迈步和走到指定目标已有用户现场确认。旧判定还要求约14°朝向精度，
到点后的原地转向会被平面进展检查判成超时；当次日志未包含末端误差，无法确认全部原因。
按用户选择，现为当前位置距目标≤30厘米即到达、朝向不限，取消末端强制转向。
更新并保留当前位置重启后重新发目标。`/operator/status` 的成功文字为
`action status=4 (SUCCEEDED); reached waypoint`，同时显示末端XY误差；
真实Nav2失败仍为 `ABORTED`，不会仅凭靠近目标改成成功。

首次运行和 mapping/localization/navigation 模式切换通过 SSH 完成；
当前没有把远程重启容器接入 Foxglove。你可将上述面板保存到现有 Foxglove 布局供以后使用。

连续多点：运动关闭时依次完整发布多个 `/goal_pose`（每点“位置＋朝向”两次点击），再启用一次。
现在每次发布都会追加，当前点真实 `SUCCEEDED` 后才发送下一点；运行中可追加。
`/operator/status` 的 `current/queued/completed/paused` 显示执行进度。
新版布局的 `/navigation/waypoints` 以编号显示待执行绿色、当前橙色、暂停红色航点。
某点失败会保留该点及尾部并暂停；“重试暂停路线”发布 `/navigation/resume`，或SSH执行
`bash ~/go2_nav/go2_3d/tools.sh resume`。启用按钮不会自动重试失败点。
取消按钮现为“取消路线”，会清空全部航点；停止运动也清空全部航点。
定位丢失保留路线并暂停，恢复后点“启用运动”继续；容器重启不保留内存中的路线。
路线完成后再次选点会形成待启动的新路线，需再次点启用。
地图命名归档与室内/农场切换见README“多点导航与室内、农场地图保存”。

下一点与当前机身方向差别较大时，官方 Rotation Shim 先原地转到新路径方向，
再交给 DWB 以原有0.30m/s配置跟踪；不要求对准 Goal Pose 点击时的最终朝向。
进展检查包含实际朝向变化，原地转身不再仅因XY没有移动而超时；完全不动仍10秒失败。
若停止在第二点前，用 `motioncheck` 的实际插件参数和 `max_heading_change` 核对
是否正在转身；遇到真实外形旋转碰撞仍会被拒绝，失败不会伪装成到达。
