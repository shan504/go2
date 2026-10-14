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
取消、停止清除等待目标；后续Pose路线改为定位失效保留并暂停。失败action不因重复启用而重试。
没有加长进展超时或跳过外形检查来掩盖缺少位移。

真实 ROS action/service 回归覆盖等待超过 10 秒、参数拒绝、待执行取消、定位失效、
启用时目标格已被占据，以及已启用时直接发送。运动桥新增 `/control/status`，
包括阻断条件、Move 请求统计/最近速度、按本桥请求 ID 关联的 Sport 响应 API/code。
隔离 ROS 模拟端点使用上游 Unitree 消息定义，验证 Move JSON 的限速、失效停止、
丢弃失效期间命令和响应关联；navcheck 验证该状态及控制器/平滑器速度读取。
这些测试没有执行真实电机运动。提前耗尽计时已被修正，当前实机是否还有零速度、
传感器门控或 Sport 执行问题，必须依据新增状态及实际位移判断。

## 非零速度已送达后的官方接口核对

后续上传的 `motioncheck` 把失败窗口完整覆盖：控制器 100 条非零 XY、平滑器 205 条，
桥自身 204 次非零 Move，最高 `vx=0.15 m/s`；配对响应 code=0，`/odom` 最大位移仅 0.062m。
因此不再把此次失败归因于速度话题未接通。遥控器能前进说明底盘有运动能力；
目前仍缺原生 mode/gait/error/velocity 和控制服务状态，不能确定是哪一个底盘条件阻断执行。

核对宇树官方仓库固定版本：

| 来源 | 固定提交 | 核对结果 |
| --- | --- | --- |
| [unitree_sdk2 Go2W 示例](https://github.com/unitreerobotics/unitree_sdk2/blob/63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36/example/go2w/go2w_sport_client.cpp) | `63096d0a` | Go2W 使用 Go2 SportClient 的 Move/StopMove |
| [unitree_sdk2_python Go2W 示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/example/go2w/high_level/go2w_sport_client.py) | `814556d1` | 同样调用 Move(vx, vy, yaw)；默认客户端不启用 lease |
| [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88) | `668d1ec5` | ROS Sport 请求 Move API1008 的 JSON x/y/z 与当前桥一致 |

Go2W 不因此需要改成另一个轮速话题。RPC code0 不等于运动证明；多个发现端点不等于已观察到覆盖。
未据此改变 lease、优先级、站立、运动模式或避障开关。

`robotcheck.py` 只允许以下服务/API 组合：`motion_switcher/1001` CheckMode、
`robot_state/1003` ServiceList、`obstacles_avoid/1002` SwitchGet，以及后续新增的 MCF `sport/1034` GetState。
API 必须与服务一起核对：
1003 在 sport 上是 StopMove，在 robot_state 上才是只读查询。回复必须同时匹配请求 ID 和 API。
动态订阅原生 SportModeState，保留 Go2W 原始枚举、实测速度及位移；缺少接口不假定为零速度。

用户明确要求官方前进命令后新增 `drivecheck.py`：先确认导航桥已禁用且处于导航配置，
手动发固定 0.15m/s、20Hz、两秒 Move1008，正常、异常或 SIGINT/SIGTERM 退出后发 Stop1003。
读取底盘状态与上述只读 RPC，隔离 Nav2 与底盘执行。它不自动切换任何底盘模式。
Humble/FastDDS 隔离 ROS 模拟验证请求、响应关联、拒绝与停止路径；未连接实物，
没有验证轮子真实响应或使用机器人上的 CycloneDDS 执行此测试。

## MCF 与开启的原生避障接口

后续实机直接 Sport 两秒测试：38 次 Move1008 和一次 Stop1003 全部响应 code0，
现场仅身体探前、没有前进；两种频率原生状态都只有约 5cm 位移。
MotionSwitcher 回复 `{'form':'0','name':'mcf'}`；原生避障 SwitchGet 回复 enable=True。
不把 `error_code=100` 解码为特定电机故障，不把旧版 mode0/gait0 作为 MCF 的完整状态解释。

官方 SDK `814556d1` 的
[ObstaclesAvoidClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/go2/obstacles_avoid/obstacles_avoid_client.py)
及[示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/example/obstacles_avoid/obstacles_avoid_move.py)
先 UseRemoteCommandFromApi(True)（API1004），再发 Move（API1003，x/y/yaw/mode=0），
发送零速度后 UseRemoteCommandFromApi(False)。这说明存在独立的接口契约，
不能仅凭避障开启就断言普通 Sport 必然失效；通过限时对比测试确认实际固件行为。

补充参考社区
[MCF 示例](https://github.com/legion1581/go2_webrtc_connect/blob/e0abc5780761539eff89a382a79319e5cf6ad1f4/examples/go2/data_channel/sportmode_mcf/sportmode_mcf.py)
固定提交 `e0abc578`：MCF 沿用 Sport Move1008，提供 GetState1034（参数为状态键数组）。
`robotcheck` 只增加该只读查询，不采用社区脚本中的模式或姿态切换。
该 GetState 查询不在当前官方 SDK SportClient 中，因此不称为官方定义，也不据其缺少回复认定故障。

新增显式 `drivecheck-native`，使用已开启的内置避障路径两秒，不调用开关避障的 SwitchSet。
所有正常、SIGINT/SIGTERM 和失败收尾发送原生零速度、Sport Stop，并释放 API 输入。
输入切换回复丢失也清理；速度拒绝和桥再次启用提前停止。保持原导航桥配置，
等待实际前进验证再决定是否把该后端用于 Nav2。隔离 ROS 模拟验证协议和失败收尾，
没有验证真实底盘运动。

## 四足确认与站立准备假设

用户确认官方遥控器让机器人四条腿迈步，纠正此前按轮足 Go2W 描述的假设。
此前 Move/Stop 的字段核对仍适用，但后续模式、步态与物理行为应按四足 Go2 固件核对。
原生避障测试的输入选择、39 次 Move/零速度和释放都响应 code0，仍只探身、位移约 7.5cm。
API1034 的 code0/data='' 不提供可解释状态，不能认定其在该固件上受支持。

官方[四足 Sport 示例](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/example/go2/high_level/go2_sport_client.py)
提供 BalanceStand；上述社区 MCF 示例明确提示 StandUp 后可能锁定站立，Move 前执行 BalanceStand1002。
这是待验证的准备状态假设，尚未证明当前机器人处于这种状态。
新增显式 `drivecheck-balanced`，不改变导航默认行为：单次 BalanceStand，匹配 code0，
等待0.5秒、重新核对禁用桥，然后保持0.15m/s两秒 Move并Stop。
匹配失败、缺少准备回复和SIGINT/SIGTERM均Stop，未获准备确认不发Move。
隔离ROS验证协议与退出路径，不据模拟成功声称机器人已能迈步。

## 平衡准备失败后：核对低速起步响应

用户实机执行 `drivecheck-balanced`：一次 BalanceStand、39 次 Move、一次 Stop 都匹配 code0，
最大原生 XY 位移约 0.086m，仍只前探、没有迈步。该准备假设没有解决当前故障，
不把 BalanceStand 自动加入导航，也不继续重复 0.15m/s 的准备动作组合。

重新在线检索并读取以下案例：

- [社区 #30](https://github.com/legion1581/unitree_webrtc_connect/issues/30)：持续发送 Move
  解决单次速度指令后只走几步的问题。本机已经 20Hz 持续发送，不符合其原因。
- [社区 #17](https://github.com/legion1581/unitree_webrtc_connect/issues/17)：StandUp 后
  Move 无效，BalanceStand 解锁。本机已验证该步骤仍不迈步，不能据此认定关节锁定。
- [官方 SDK 仓库 #175](https://github.com/unitreerobotics/unitree_sdk2_python/issues/175)：
  用户报告 StandUp 后暂不响应；没有官方就绪状态解答。`_CallNoReply` 本地返回0与本项目
  按 Request ID/API 配对的回复0不同，两者都不等于实际行走。
- [另一台 Go2 的实机 #26](https://github.com/armwaheed/mappo-arm-cloud-physical-ai/issues/26)：
  持续约0.137m/s停滞，约0.295m/s行走3m；原先猜测0.35阈值已被其记录反驳。
  [#42](https://github.com/armwaheed/mappo-arm-cloud-physical-ai/issues/42) 还说明不同方向、
  从站立起步和已经迈步时的响应不同。负载、温度、位置和固件是混杂因素，阈值不通用。

本项目将导航和所有手动测试都限制在0.15m/s，此前遗漏了起步速度这一变量。
官方固定提交 `814556d1` 的 Go2 Sport 示例向前为 `Move(0.3,0,0)`。
新增显式 `drivecheck-walk`：固定0.30m/s、20Hz、最多两秒、Stop配对确认；
不改变姿态、步态、控制权或避障设置，不修改导航速度上限，不做速度后置放大。
Move前及期间检查新鲜 odom，记录命令期间的净位移与起始朝向投影；
Stop缺失确认、Move非零错误、传感器超时、SIGINT/SIGTERM和桥再次启用均有明确退出路径。
模拟测试验证这些路径及“全回复0但里程计不变”的情况。

## 0.30m/s 实机迈步成功后：统一导航速度链

用户运行 `drivecheck-walk` 并明确确认向前行走。39次 Sport Move1008 和 Stop1003
均配对 code0；Move期间2秒 `/odom` 净XY位移0.424m、沿起始方向前进0.423m、侧向0.025m。
运动模式MCF、原生mode/gait=0、error_code=100和内置避障启用状态与失败测试一致。
因此此次无需更换接口或自动调整姿态/步态；100不是本记录中无法行走的判据。
这个结果只验证当前机器本次能够以0.30m/s起步，不确定所有机器的最低起步阈值。

`motion_profile.py` 统一三维导航DWB、平滑器、桥的0.30m/s前进上限，角速度仍0.30rad/s。
仅改上限仍会让DWB持续选择低速，因此 StandardTrajectoryGenerator 使用两个X样本
（0/0.30）、线加减速度0.40m/s²和1.2秒预测，从静止也能提供0.30的有效前进样本。
原地转向及停止仍有0速度样本；平滑器完成加减速，不在执行器后端放大规划指令。
RotateToGoal与实际XY目标容差对齐，避免提前进入只转向窗口。
上游未修改的 OneDVelocityIterator 在反向、静止、低速及超限初始速度下均验证X样本0/0.30。
实际ROS接口回归验证0.30到达Sport、0.08不被放大、失效Stop和恢复不重放。
这些模拟与采样检查不能证明完整Nav2目标已在实机到达。

新 `restart-navigation` 显式命令先禁用运动和清除旧目标，只读获取新鲜有效的GICP map位姿，
通过有限数值参数传递XYZ和yaw重启。读取失败不停止原容器；重启期间保持机器人静止。
该命令解决已前进42厘米后不能再用原点初始化的问题，不重置地图、不自动启用或重发目标。
真实ROS TF回归覆盖有效位姿、无效定位及过期TF拒绝；隔离shell回归核对启动参数。

## 实机到点但进展超时：巡检只要求位置到达

用户后续确认已走到指定目标，日志仍为 `Failed to make progress`、Nav2 ABORTED。
Humble SimpleGoalChecker 先检查XY，再检查yaw；原SDK位置容差0.25m、yaw容差0.25rad。
SimpleProgressChecker只看平面位移，原地转向不算进展，10秒后会抛出该错误。
截图没有末端位姿数据，不能仅凭截图断定朝向是唯一原因。

用户明确选择“到指定位置就算成功，终点朝向不限”。配置现将SimpleGoalChecker设为
XY半径0.30m、yaw=π、stateful=false；其角差取最短路径，范围[-π,π]，因此任何朝向均满足。
每次检查当前XY而非锁存曾经进入的范围；DWB不再加载RotateToGoal。
行为树显式传goal_checker_id，消除未指定判定器警告。目标外进展计时保留，不伪造成功。

`tests/goal_checker_smoke.py` 使用生成参数调用实际SimpleGoalChecker：实机镜像链接已安装
官方库；云端编译未修改的上游实现、原版nav2_core/utility头文件、实际rclcpp参数和TF数学。
云端仅为未使用的Costmap2DROS参数提供类型前置声明，未模拟到达算法。
angles头文件来自官方ros/angles提交 `a96224f9ab3ac51fe8fd981c1e1554528dc4345a`。
回归复现旧配置拒绝位置内90°朝向，验证任意朝向的30厘米内成功、边界外/远处/对角距离超限
拒绝、目标重置。ROS action回归核对真正的SUCCEEDED/ABORTED文字和末端XY误差，
近点ABORTED不会被重写成成功。用户随后提供实机 `Reached the goal!`、`Goal succeeded`、
`action status=4 (SUCCEEDED)`，末端XY误差0.267m，已确认新到达规则在实机生效。

## 顺序航点与独立地图版本

用户后续实机确认预测修复后仍在第二点暂停：第一点SUCCEEDED、误差0.275m，
第二点自动接受，约3秒后因0.091m/0.88°静止校正被旧0.05m/2°限制取消；
静止时还反复拒绝0.059--0.101m校正。用户明确要求恢复室内最近0.5秒、约8帧累积，
并放宽用于150米环形石子路巡检的定位。对照室内多点版本e3ab6a9，速度/到点和
已验证顺序队列保留；此前单帧优化取消，默认保留位姿重启恢复dense。
定位使用0.10m降采样、0.30m匹配距离、最低fitness0.30、最大RMSE0.15m；静止相邻
校正限度为0.25m/8°。保存PCD不改，建图仍使用原质量阈值及0.05m/约2°静止限制。
缺帧/低质量匹配在上一份已接受扫描的原1秒期限内保留估计，不续期失败扫描；
累积窗口中的空新包也不能刷新期限，按最新真正贡献几何的原始receipt/stamp检查新鲜度。
非有限/过大创新、过龄结果及超限静止校正仍立即失效。Open3D拒绝原因区分质量与创新。
实际ROS回归以约16Hz、每包1500个不同采样点验证0.5秒约8帧累积，7.6/9.1/10.1cm
及1.89°波动保持连续有效，空包不续期，持续缺失到期停止TF；28cm/10°单次超限
校正拒绝，PCD字节不变。历史严格场景显式使用旧精度/阈值继续验证。
这些是合成传感器回归，不是农场原始扫描回放或Jetson耗时验证。大片空地缺少地标时
仍可能出现平面位置不可观测/错误对应；当前GICP前端没有回环，不能据此保证150米闭环。

最新实机速度采集已证实控制器输出0.30m/s，但平滑器只升到0.12m/s、所观察Sport请求
只到0.06m/s，约0.3秒后因定位失效取消。速度配置/桥与室内实际SUCCEEDED版本80ed81b相同。
随后的定位状态为285007地图点、8帧约1.2万输入点/6千细配准点，fitness约1.0、rmse约0.026；
点云年龄0.00/0.01秒，但接受匹配年龄1.04/1.05秒，已经超过原1秒期限。
这证明源数据仍到达时，配准结果未能持续保持新鲜；不能通过提高速度上限解决。
该次尝试曾将实时定位/导航的scan_window覆盖为0.0；现已按上述用户要求恢复0.5秒。完整保存PCD
照常加载。该尝试保留1秒期限、0.30m/s和地图文件；最新参数以本节开头为准。
motioncheck补齐真实GICP参数、有效性边沿、完整失效原因和最近成功的处理/源年龄。
真实ROS launch回归核对standard/dense全部模式的参数优先级；模拟传感器回归验证稠密建图
保存后单帧细定位、全PCD加载、地图字节不变、真实异常及陈旧输入仍停止TF。
这些云端验证不代表Jetson上的单帧处理耗时或完整实机路线已验收。

后续用户日志中目标已执行但约0.24秒后因定位失效暂停，完整GICP日志只有
`Waiting for odom -> base_link at cloud timestamp`，未见该次启动的配准跳变拒绝。
已用真实ROS复现点云只发一次且先于TF到达、TF随后可用时，旧回调仍丢弃该点云。
单独恢复时间戳TF等待重试，不恢复Point输入：最多32包、50ms非阻塞重试，仍按原始
扫描时间查询TF，保留最初接收时间，超过原有1秒窗口的包丢弃；晚到TF不能重新激活过期包。
状态提供接收/对齐/TF等待/过期计数、配准处理耗时和原始数据年龄。
少量等待日志不等于所有定位中断均由该顺序引起，修复后的实机持续定位仍需现场确认。

后续用户日志确认单点真实SUCCEEDED，随后出现静止GICP修正跳变拒绝和导航取消。
Pose路线现为显式收集后启用：只订阅 `/goal_pose`，不依赖 `/clicked_point`。
新路线不会继承上一轮已开启的桥状态而提前出发。定位失效保留当前点与尾部并暂停，
动作取消与底盘新鲜定位门控仍保留；恢复后需显式启用，不自动出发。
实际规划/控制失败仍暂停且需显式重试，Stop/Cancel仍清空。MarkerArray完整快照每秒重发。
新增隔离真实ROS回归覆盖纯Pose三点一次启用、逐点真实成功、已启用桥上的新路线收集、
定位暂停/取消中晚到的接受响应、恢复后的手动启用、失败不跳过及面板重连后的全部编号。
另用真实GICP复现移动时接受10cm修正、平滑到2.5cm后停下，旧实现把相同测量的剩余7.5cm
误判为新的静止跳变，持续拒绝定位。现将静止检测与前次已接受的原始配准修正比较，
保留5cm/2度限制和显示平滑；拒绝的新测量不会污染参考值，显式重定位会重置参考。
回归同时检查稳定收敛、真实15cm静止跳变拒绝及恢复。
该验证不代替实机多点路线验收，0.30m/s、5cm膨胀、GICP阈值与保存地图保持原样。

单帧更新后，用户确认第一点到达；第二点日志为current=2、completed=1、Nav2已接受，
随后拒绝5.5/7.4/7.0cm静止校正而取消。队列已经顺序发送，阻断位置为定位检查。
代码仍使用输出滤波后的correction预测下一次ICP。实际ROS/Open3D回归复现：有结构的扫描
先接受10cm里程计校正，输出平滑只走到2.5cm；停步后的重复地面稀疏扫描从该滞后预测
落入另一组对应点，生成10cm错误跳变并失效。改为accepted_match_correction乘当前
时间戳里程计作为配准预测，输出平滑独立保留，bootstrap/建图初始化不变。
同一个真实GICP回归现在通过稀疏停步和正常平滑收敛；故意制造真实5.5cm、7.4cm、15cm
静止矛盾仍拒绝，且拒绝结果不更新预测参考。没有增加帧窗口或改变1秒新鲜期限。
该场景是合成传感器回归，不是用户第二点的原始扫描回放；现场第二点仍需复测确认。

多点继续使用已经实机成功的 `NavigateToPose`，由操作节点排队，而非并发发送多个action。
每个点真实SUCCEEDED后重新读取运动开关，并检查最新定位、TF和代价栅格，再发送下一点。
ABORTED/拒绝/下一点失效时保留失败点及尾部，显式resume才重试；停止、取消清空。
定位丢失保留并暂停，恢复后显式启用继续。
实际ROS action/service回归验证顺序执行、执行中追加、桥被外部禁用、失败重试、下一点变障碍、
尚未接受目标时取消以及定位丢失；最多一个执行中的action，多点完整实机路线仍需现场验证。

地图为 `maps/library/名称/版本/` 完整bundle，室内归档复制原件并验证SHA256，
原点、PCD、栅格及定位参数不变。新农场保存使用独立名称和新时间戳，任何旧版本均不覆盖。
归档不改变当前加载地图；切换拒绝运行中的容器。单元回归覆盖多版本/幂等归档、复制中断、
无效路径、历史版本切换、命名图清理仍保留PCD；ROS模拟传感器验证标准/稠密地图保存和重载定位。

## 第二段大角度起步转向（2026-10-12）

本次用户实机日志为第一点SUCCEEDED、误差0.276m，第二点自动发送且接受，
随后10秒 `Failed to make progress`；没有定位丢失取消。用户确认第二段基本不转身。
这个日志证明队列和动作切换已执行，不包含当时完整速度/代价格数据，不能仅凭它
断言具体是哪条DWB轨迹打分造成了不动。

查阅并核对在线官方Humble分支与本地检出一致（3c3db59）：
- [Rotation Shim README](https://github.com/ros-navigation/navigation2/blob/humble/nav2_rotation_shim_controller/README.md)
  明确列出DWB新路径起步时大方向差可能不能干净地原地旋转，建议由shim先转向再交接。
- [SimpleProgressChecker](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/plugins/simple_progress_checker.cpp)
  只计算XY距离；[PoseProgressChecker](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/plugins/pose_progress_checker.cpp)
  加入最短角距离进展。0.30rad/s原地转180°需要超过10秒，不能只靠XY进展判定。

配置采用官方RotationShimController包装原DWB，20°触发、11.5°交接，
角速度0.30、角加速度0.60，closed_loop=false从上一条命令递增，不依赖尚未转身时
接近零的实测yaw速度。仍保留官方旋转外形碰撞检查和DWB的真实外形检查。
PoseProgressChecker用10cm或约11.5°作为进展，10秒实际停滞超时不变；
不修改定位、地图、膨胀、终点位置容差或FIFO，失败不自动跳过。
启动前检查插件XML及开环字段；旧镜像缺支持明确要求重建，而非悄悄忽略参数。

验证编译未改动的官方Humble shim、TF转换、真实Costmap2D/外形碰撞算法及进展检查。
云环境只适配Costmap2DROS生命周期包装，路径跟踪交接用确定性测试观察器，
不把它当作完整DWB或实机验证。覆盖左右126°/179°路径、每秒重规划、零实测角速度下
的命令递增与0.30m/s交接；复现旧进展超时、新进展识别真实转动，仍拒绝停滞/微抖/角度绕回。
致命障碍和跟踪未知区域会拒绝旋转。完整第二段仍需用户实机复测。

## 新旧地图统一使用公共修复（2026-10-13）

用户确认当前版本已成功，要求新建和切换地图继续使用这一版的公共修复。
检查发现速度/转向/到点判定、Pose队列、时间戳TF重试及GICP预测已经是公共代码；
缺口在新图默认10厘米栅格、离线处理只修改选中的历史版本，以及实时障碍高度来自旧地图参数。

新图默认保存5厘米栅格。每次导航启动在停止传感器前重新准备公共Nav2配置和
`runtime/navigation_map/`：读取完整所选PCD，保留地图坐标与地面模型，按当前
`gicp.yaml` 的栅格/高度和 `navigation_filter.yaml` 的去噪规则生成运行栅格。
已知自由区域按面积转移，旧版调平图自动从保留祖先恢复自由面积；旧图无地面模型时
在运行副本中估计。Nav2静态地图和实时障碍过滤使用同一地面与当前策略。
启动不改原PCD、元数据、地图版本或latest链接；失败不停止原容器，不启用运动。
地图归档、修复、清理和可选地面对齐仍可显式另存版本。

Open3D 0.19/0.14算法回归覆盖跨图切换、重新应用全局过滤、粗细栅格面积转换、
旧调平图自由区域恢复、原图SHA256不变与失败保留运行栅格。
真实ROS2 Humble验证公共Nav2配置、所有模式/standard+dense参数、导航准备入口和
实时障碍节点忽略旧高度设置；合成传感器建图/重载定位验证5厘米保存与原定位行为。
启动shell回归验证准备失败不停止传感器；导出包附带实际运行栅格以便核对。
这些验证未连接实物，不代表新增场地的完整路线已实机验收。

## 建图端恢复成功版本，导航策略单独配置（2026-10-13）

用户在 `mapping dense farm2` 报告整张地图移动。现场诊断确认 `/map_cloud` 有85213点、
`frame=map`；里程计静止位置范围不到1毫米，GICP `map→odom` 单次校正变化4.83厘米，
Y方向范围6.19厘米。配准处理0.127秒、源年龄0.147秒，未见扫描过期；建图模式的
`/localization/valid=false` 是禁止导航运动的正常约定。二维 `/map` 由导航模式发布。

上次把建图原始观测栅格从10厘米改成5厘米超出用户只扩展公共修复的要求，现恢复。
`mapper.py`、`gicp.yaml`、`gicp_dense.yaml` 与已成功的 `cd1116c` 完全一致；
未根据这一份诊断就把校正波动归因于栅格分辨率，也未新增配准滤波或放宽跳变阈值。
统一导航栅格/高度改由 `navigation_grid.yaml` 配置，所有地图仍在导航启动时生成5厘米运行栅格。
真实ROS2回归验证standard/dense均保存原始10厘米栅格，运行导航副本为5厘米，PCD字节不变。

整张地图的显示移动仍需核对Foxglove固定/显示参考系为map、关闭跟随/时间同步、
只显示 `/map_cloud` 后是否存在；`map→odom` 波动与当前视角的关系不能从单张截图确定。

## 确认固定map后坐标仍移动：静止建图保持锚点（2026-10-13）

用户进一步确认固定/显示参考系均为map、仅显示map_cloud后仍移动，并明确指出机器狗坐标系移动。
恢复旧版代码不足以解决该现场波动：原建图直接输出每次通过5厘米跳变检查的GICP校正，
静止时仍可逐次改变map→odom并用不同位姿积累扫描。

建图现在仅在连续真实里程计变化小于2毫米/0.002rad时保持已建立的坐标锚点。
新扫描仍做GICP、创新/静止跳变及新鲜度检查；额外验证实际锚点位姿下的重叠率/RMSE，
合格才以该锚点积累扫描、发布registered_cloud与TF。大于原5厘米/约2°的矛盾仍拒绝；
运动时正常更新GICP校正，实际odometry仍直接参与机身位姿。定位模式沿用原已接受预测与输出平滑。
状态新增mapping_anchor=held/updated，便于现场确认实际分支。

未修改的恢复版本在真实ROS/Open3D回归中复现静止扫描扰动使坐标平移2.39厘米。
修复后standard/dense回归的原始GICP校正跨度分别为4.8/6.0厘米，输出锚点与ROS TF保持稳定，
真实模拟走动/转向仍校正里程计偏差；15厘米静止矛盾拒绝、地图不污染且正常扫描可恢复。
42项算法测试、标准/稠密建图保存及重载定位、原定位停步预测回归均通过。
这是合成传感器验证，不是用户原始点云回放；实机仍需更新后确认。

## 所有地图共用完整地面坐标处理（2026-10-13）

用户再次报告三维点云、二维地图和机身分层。此前公共处理只重建导航栅格，
定位仍读取原始机身原点PCD，完整地面对齐仅通过逐图执行level-map生效，这一遗漏已修复。
新地图保存、旧地图定位和导航均调用同一build_bundle：完整PCD和已观测自由区域一起做
刚体变换，地面为z=0，真实机身离地高度写入统一元数据。地图服务、定位和实时障碍
读取同一个完整运行副本；已对齐地图不会重复抬高。原始保存版本保留，构建失败不改变latest。
保留当前位置重启时，旧坐标的位姿同步变换到公共地面坐标。未来全局修改要求写入根AGENTS.md。

45项算法测试通过。真实ROS2检查覆盖原始旧图、已对齐图、不同名称的倾斜地面图，
定位初始化的机身离地高度分别约0.400/0.400/0.459米，导航footprint均为z=0；
倾斜图保留35996个完整三维点与点间距离。重复加载结果稳定，已对齐PCD字节不变。
标准/稠密建图保存、定位重载、静止锚点、公共启动参数和保留位姿变换检查均通过。
这些检查使用合成数据和真实ROS2节点，未连接用户机器狗；实机显示需更新后确认。

## 公共航点收集与导航执行分开（2026-10-14）

用户报告Goal Pose不动、只能设置一个点，要求恢复多点导航并对所有地图生效。
队列仍支持100点，但此前收集输入就检查Nav2 action、实时TF和代价地图，启动或短暂
未就绪时会直接拒绝点击。现在收集仅检查map参考系、有限合法位姿和静态地图自由格；
启动及每次发送下一点仍做完整实时定位、TF、新鲜代价地图、障碍和运动开关检查。
旧布局的明确PointStamped输入/navigation/waypoint恢复并进入同一队列；无关的
/clicked_point工具不触发导航。新路线仍需先收集再启用一次，不自动开始运动。
公共操作节点不依赖地图名称、保存版本或原点，未来修复要求继续由根AGENTS.md约束。

旧代码在隔离ROS回归中复现Nav2未就绪丢弃首个Pose输入；修复后Pose/旧Point混合
三个编号航点在定位、action、TF、代价地图尚未就绪时均保留，就绪后一次Enable按顺序完成。
缺失或过期TF/代价地图、实时障碍值99均阻止执行并保留路线；两种地图原点、范围和
分辨率下的静态障碍/未知格拒绝仍有效。原有队列、Pose路线、启停取消、定位暂停恢复、
下一点障碍复查、真实action成功/失败测试及45项算法检查通过。测试使用真实ROS服务与
合成action端点，未执行完整Nav2规划或连接实物；用户当前实机不动的阻挡环节仍需motioncheck输出确认。
浏览器保存的布局不会随git pull替换；全部航点应显示/navigation/waypoints编号，
/goal_pose自身只显示最新箭头，Pose输入每点需完成位置与朝向两次点击。

## 所选地图起点自动初始化（2026-10-14）

用户已通过发布初始位姿确认定位和导航成功，并明确每次启动前都会把机器狗放回建图起点。
公共shell、ROS launch和定位节点现在默认自动使用所选地图的起点，不再要求鼠标initialpose。
新保存地图记录首次已接受建图扫描的完整map_start_pose，保留初始机身roll/pitch；
保存、地面对齐和运行副本均将起点与PCD乘以同一个刚体变换，已对齐地图重复加载不重复变换。
旧地图按机身原点约定及保留的地面对齐变换链恢复起点，包括归档中的/maps路径。
缺少原始姿态记录的旧图只能提供起点提示，仍由实际GICP确定完整姿态；不凭提示发布有效定位。
显式X/Y/YAW/[Z]和保留当前位置重启覆盖自动起点，手动initialpose仍可重定位。
新到达的有效手动提示优先于自动配置，点云/精确时间戳TF尚未就绪时保留最多10秒。
Nav2启动仍要求真实有效GICP和新鲜TF，运动仍默认关闭。起点自动行为及全局修改要求写入AGENTS.md。

51项算法测试通过。真实ROS验证两种地图、完整机身姿态、原始/已对齐/旧图变换链及重复加载：
未发布initialpose时自动接受细GICP，机身姿态与独立计算的起点相符，footprint为地面z=0；
坏扫描和缺失TF不能激活定位或生命周期。标准/稠密建图实测保存首帧起点，后续移动不改变它。
公共launcher标准/稠密及所有模式、显式坐标覆盖和手动关闭自动初始化均通过真实ROS参数检查。
手动早到提示、过期提示、重定位失效与恢复、点云先于TF、原建图保存/重载定位回归通过。
启动日志同时显示具体initialpose/点云/TF/GICP原因与状态年龄，过期状态文字不能授权启动。
以上使用真实ROS与合成扫描及生命周期记录服务，未连接用户机器狗或运行完整Nav2规划。
