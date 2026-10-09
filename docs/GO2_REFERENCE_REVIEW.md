# Go2 建图与导航实现对照

2026-10-08 读取公开仓库和 ROS2 Humble Nav2 源码后，对照本项目实机输出。
下面是源码证据；没有把网上演示当作本机验证结果。

| 项目 | 读取的提交 | 对当前机器狗有用的部分 | 适用范围 |
| --- | --- | --- | --- |
| [andy-zhuo-02/go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox) | `cd11d747` | 原装 Go2 EDU 扩展电脑、Foxy、`cloud_deskewed`/`robot_pose`、点云积累、Sport Move/Stop | 导航定位采用二维 slam_toolbox；其时间戳重写和固定高度不能直接替代三维 GICP 链路 |
| [zhuaoyuRobo/Go2W-navigate-project](https://github.com/zhuaoyuRobo/Go2W-navigate-project) | `edf63bbc` | FAST-LIO 建图/里程计、PCD 定位、NDT/GICP、Nav2、机身点云过滤分开处理 | Go2W、Livox/其他雷达接口；MID360 的驱动、IMU 外参不能套到原装 Go2 EDU L1 |
| [h-naderi/unitree-go2-slam-nav2](https://github.com/h-naderi/unitree-go2-slam-nav2) | `34ba48bb` | Go2 Sport 控制、Nav2 代价地图/参考系的集成示例 | 外接 RoboSense 雷达、二维定位；其 0.55 m 膨胀参数不是本机建议 |
| [Nav2 Humble navigate_to_pose.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp) | `3c3db59d` | `configure()` 直接订阅 `goal_pose`；`onGoalPoseReceived()` 不经本项目检查即发 action | 已据此隔离重复目标入口 |

## 本次实机证据

目标 `(0.19,0.66)` 在修复后的原始 `/map` 中是 100，全局代价地图也是 100，局部是 99。
因此仅减小膨胀半径不能让该点可达。当前包附近大部分回波距地面约零，仍有三个障碍回波，
最高约 0.266 m；尚不能确认这些回波是真障碍、机身回波还是已有地图的残影。
截图中 `/localization/valid=false`，启用操作被拒绝；TF 报错引用约 94 秒以前的 `base_link` 位姿。
当前不具备宣称实机路径规划及运动已成功的证据。

## 已修正

- Nav2 内置目标话题单独重映射；Foxglove 点击只走操作节点，固定 map 目标不再携带历史机身时间。
- 发送 action 前检查新鲜定位/TF、实际地图及全局/局部目标格子；拒绝原因保留到状态话题。
- 全局导航轮廓使用独立的平面 `base_footprint`，保持真实 XY/yaw；真实三维机身姿态不变。
- 新建图的首帧只归零位置/朝向，保留 odom Z 轴，避免首帧机身倾斜被写入地图轴方向。
- 规划失败直接结束，不使用默认恢复树的 Spin/BackUp；定位失效显示具体原因/数据年龄。
- ARM64 启动限制 OpenMP 为两线程、BLAS 为一线程，减少注册算法与导航并行时的线程争用。

这些修改不会自动修复旧 PCD 的重影或使障碍格变成自由格。
现有地图先通过 `tools.sh export-map` 导出，离线查看点云、地面残差及占据单元对应的实际几何；
如果是配准漂移，需要重新构建地图或使用具备正确 L1 点时间/IMU 外参的 LiDAR-inertial 前端。
原装已经提供 `/utlidar/cloud_deskewed`，后续优先验证其 odom 参考系和时间契约再接入。
当前 GICP 仍用于三维 PCD 地图定位，导航仍为 Nav2，不切换到 AMCL。

## 2026-10-09 实际 PCD 检查与后续路线

已读取用户上传的 `go2-map-debug.zip`，PCD 含 163426 点。目标附近的高点连接成连续表面，
用户确认附近有椅子腿等物体；不再把它们当孤立噪点删除，也不通过缩小外形强行开放目标。
用户决定改在宽阔场地重新建图，下一步不继续围绕旧障碍布局试目标。

原图地面模型倾斜约 2.4°，起点地面位于机身原点下方约 0.384 米。
新增离线 `level-map`：刚体变换全部 PCD 点，使同一拟合地面成为 z=0；同步重建 5 厘米栅格，
保留原目录并保存 `map_from_previous_map`、`body_height_above_ground`，不会虚构自由空间。
定位节点按新图保存的高度解释二维初始位姿；真实 3D 机身 TF 和平面导航轮廓保持分离。
原上传地图的刚体对齐已执行验证，全点数保留；由实际 PCD 采样的合成扫描通过真实 ROS GICP
定位与机身/轮廓 TF 检查。不是原始传感器回放，未据此声称旧图或新场地已可导航。

## 验证范围

在云端 x86_64 ROS2 Humble 中验证算法、生成 URDF/配置、上游启动条件、真实 ROS TF、
真实 NavigateToPose action 协议及生命周期服务协议，使用合成传感器和测试 action/service server。
没有在云端验证真实 Go2 运动，也没有运行实际 Nav2 规划器来证明当前室内目标可达。

## 细栅格自由区域与本次规划失败

随后实机输出：GICP 有效，目标 `(1.66,1.77)` 在三张栅格中均为自由，静态及局部格子连通，
全局代价地图格子不连通。Nav2 接受 action 后立即规划失败，status=6；启用控制只在等待新 cmd_vel。
这证明当前失败在全局路线，不能把它解释成运动开关未接通，或仅凭目标中心/照片认定能规划。

检查发现上一版细栅格转换只保留粗自由格的中心子格，丢失其已知面积。
已改为保守面积转换：细格的旧栅格覆盖区域必须全部已知自由，未知/障碍不会因此被清除。
`repair-map` 可从保留的祖先粗栅格恢复面积，当前 PCD 字节不变；既有 GICP 坐标不重新对齐。
0.23 米真实内切净空保留，额外软缓冲改为 0.02 米，总膨胀半径 0.25 米。
新增静态内切净空对照，进一步区分物理净空/未知断路和实时全局障碍标记断路。

回归直接编译上游未修改的 Humble NavFn 核心，对同一稀疏地面地图，旧转换得到零路径步数，
面积修复后得到 38 步路径，端点均为自由格。这里验证了转换缺陷及修复，不是新实机地图的路径保证。
用户先前上传的 163426 点 PCD 另经过栅格修复，验证文件 SHA256 不变。
当前 153401 点新图是否恢复全局通路，仍需在实机重新载入修复栅格验证。

## 2026-10-09 小连通障碍去噪

用户确认希望忽略独立的小块，包括将来农场的小障碍。读取 Nav2 官方 GitHub 的
[Humble DenoiseLayer 源码](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_costmap_2d/plugins/denoise_layer.cpp)、
[图像连通块算法](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_costmap_2d/include/nav2_costmap_2d/denoise/image_processing.hpp)
以及对应单元测试。官方网页文档和 PCL 教程在云端返回代理 403，本次结论来自成功读取的仓库源码，
没有假称读取了受阻网页。当前 Humble 分支已包含这个插件，旧镜像则可能没有。

`minimal_group_size` 定义保留块的最少格数；`group_connectivity_type` 可为 4 或 8。
插件对 `LETHAL_OBSTACLE` / `INSCRIBED_INFLATED_OBSTACLE` 做连通块过滤，
小于阈值的块改为 `FREE_SPACE`。因此必须放在障碍标记之后、膨胀之前，
并同时用于全局和局部，避免实时障碍层重新制造小块膨胀区。
它按占据格判断，不分析三维高度或识别石块、椅子腿，真实细小物体也可能被去掉。

按用户补充的农场需求，采用最小保留 8 格、八方向连通，删除独立 1～7 格障碍。
参数集中在 `navigation_filter.yaml`，
可提高阈值；不会删除整张障碍层、缩小机器人外形或把所有未知区域当作空地。
静态 PGM 用相同规则清理，新保存、修复及对齐地图均应用；现有图增加 `clean-map`，
保留原目录、PCD 字节、原坐标。这样 `/map` 中的小块不会继续使操作节点拒绝目标。
镜像构建及导航预检查均核对插件导出信息，缺失时在启动前明确报错并给出更新命令。

直接编译上述源码的 `GroupsRemover` 和未修改的 NavFn A* 核心：
0.6 米通道内的一个单格块和一个双格块，在 0.23 米内切净空下使路径步数为 0；
去噪后得到 98 步，离线 PGM 清理也得到相同结果；换成相连九格障碍仍无路。
未知格不改变。四项新增测试验证阈值、斜向连通、备份/PCD 不变和失败时原图保留，
31 项算法测试在 Open3D 0.19 / Ubuntu 0.14 均通过，配置及真实 ROS 接口回归通过。
用户之前上传的旧图删除 37 个小块、59 格，PCD SHA256 不变；不能据此保证当前新图或
农场已经可以导航。云端没有完整 Nav2 二进制包，验证范围为上游算法核心、生成配置及 ROS 接口，
并非完整 pluginlib/costmap/planner/controller 的实机运行。

## 同日：按用户要求强制总半径 5 厘米

后续实机日志中，Nav2 接受目标，控制器持续收到路径，10 秒后报 `Failed to make progress`。
这与前面的全局无路径不同；目标已到执行阶段。运动在目标后约 5 秒才启用，但仅凭这些日志
不能断言这是全部原因，也不能确认是否输出了非零速度。

全局、局部总 `inflation_radius` 都改为 `0.05`，不再加 0.23 米内切半径。
查阅同一上游 `InflationLayer::dynamicParametersCallback`：接受新半径后刷新缓存并要求重算。
新增 `inflation-5cm` 通过真实 ROS 参数服务修改两张运行中的地图，并分别回读确认 0.05；
启动配置同时固定为 0.05。真实轮廓和 DWB 外形碰撞检查保留。
上游 `onFootprintChanged` 会对半径小于内切半径发出提示；栅格量化也仍适用。
因此没有据此宣称运动故障已修复或这条真实路线必定可执行。

源码核对：Humble 控制器输出 `cmd_vel_nav`，速度平滑器输出 `cmd_vel`，运动桥订阅 `cmd_vel`。
只读诊断新增这两个话题的消息/非零计数与速度峰值，以及 `/odom` 位移，
后续以实际数据判断命令停在哪个环节。生成配置与真实 ROS 参数协议、速度诊断回归验证通过；
没有根据截图直接修改运动模式、强制发送裸速度或取消定位新鲜度检查。

## 目标等待与速度执行

最新实机日志在 `1791530979.567` 开始控制，`1791530987.766` 才启用运动，
`1791530989.569` 报进展超时：禁用运动占用了约 8.2 秒的 10 秒窗口。
操作节点现在暂存运动禁用期间选中的地图目标，读取运动桥的真实 `enable_control`，
启用写入成功且回读为 true 后重新检查定位、TF 与地图格，再发送 NavigateToPose。
取消、停止、定位失效清除等待目标；失败 action 不因重复启用而重试。
没有加长进展超时或跳过外形检查来掩盖缺少位移。

真实 ROS action/service 回归覆盖等待超过 10 秒、参数拒绝、待执行取消、定位失效、
启用时目标格已被占据，以及已启用时直接发送。运动桥新增 `/control/status`，
包括阻断条件、Move 请求统计/最近速度、按本桥请求 ID 关联的 Sport 响应 API/code。
隔离 ROS 模拟端点使用上游 Unitree 消息定义，验证 Move JSON 的限速、失效停止、
丢弃失效期间命令和响应关联；navcheck 验证该状态及控制器/平滑器速度读取。
这些测试没有执行真实电机运动。提前耗尽计时已被修正，当前实机是否还有零速度、
传感器门控或 Sport 执行问题，必须依据新增状态及实际位移判断。
