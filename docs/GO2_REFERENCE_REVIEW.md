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
