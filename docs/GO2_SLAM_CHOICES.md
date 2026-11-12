# Go2 农场巡检 SLAM 候选核查

核查日期：2026-11-12。当前已知硬件为 Go2 EDU、自带 L1、Jetson；没有使用 ZED2i。
以下依据公开项目 README、配置和作者说明，不是使用量统计，也不是同一农场的横向精度测评。
不能据此宣称某个算法“最常用”或“效果最好”。

## 与现有硬件相关的候选

| 方案 | 查到的直接依据 | 对本项目的判断 |
|---|---|---|
| **Point-LIO 的宇树适配版** | [宇树官方仓库](https://github.com/unitreerobotics/point_lio_unilidar)明确支持 L1/L2，提供 L1 演示和 rosbag | 保留 L1 时优先作为对比基线。文档测试环境为 Ubuntu 20.04／ROS1 Noetic，不能当成 Humble 即插即用；仍需闭环后端 |
| **SLAM Toolbox＋Nav2** | [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk)集成 SLAM Toolbox；[go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox)说明 Go2 EDU 实测 | 原装 Go2 有多份公开集成，ROS2接入方便，但主要是2D方案，室内演示不能证明坡地和稀疏树行下效果 |
| **FAST-LIO2** | [原作者仓库](https://github.com/hku-mars/FAST_LIO)；[Go2实机 FLMap.3](https://github.com/cj126050/FLMap.3)使用 Mid360 与 Orin Nano | 成熟前端候选，但该Go2案例的雷达不是 L1，也不是200米农场巡检验收；原版不包含完整闭环后端 |
| **LIO-SAM** | [原作者 ROS2 分支](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)说明 Humble 支持；[Go2集成案例](https://github.com/vedsom/GO_2_Autonomous_navigation)使用 Hesai XT16 | 不能因知名度就视为 L1 首选。需核对逐点时间、ring及IMU条件；原作者将自带闭环称为 proof of concept |
| **RTAB-Map 激光后端** | [Go2 L1＋RTAB-Map](https://github.com/leesj24601/Go2_L1_Lidar_Rtabmap)有 Humble 建图演示，使用 deskewed cloud 和 robot odom | 可作为全局后端候选，但该案例仍将 Nav2 集成列为后续工作，没有农场成功率证据 |

若以后更换 Mid360，可重点比较 FAST-LIO2 配合闭环后端。例如
[FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)使用 FAST-LIO2＋Scan Context／GTSAM，
保存关键帧并按优化位姿重建地图；其示例和版本背景较早，不能称为当前最新的 Go2 L1 成品。

[GLIM](https://github.com/koide3/glim)支持 ROS2、非重复扫描雷达，并记录 Jetson Orin／JetPack 6.1
测试；[glim_ext](https://github.com/koide3/glim_ext)提供 Scan Context 扩展，值得作为另一个候选。
但本次没有找到足够证据证明 L1 在当前 Go2 镜像中直接可用，实际 Jetson 型号及运行负载也需确认。

## 不能省略的接入条件

宇树 Point-LIO 配置使用 `/unilidar/cloud` 与 `/unilidar/imu`、专用 UNILIDAR 点类型。
本项目此前诊断中的 `/utlidar/cloud_base` 已在 `base_link` 下，不能直接套用原始雷达坐标的外参。
需要核对逐点时间单位、原始/去畸变点云、雷达内部IMU与机身IMU的区别、时间同步及外参。
[Point-LIO](https://github.com/hku-mars/Point-LIO)与 FAST-LIO 原作者均强调这些输入条件。

GICP 是配准算法，可用于定位，也可构成建图前端；GLIM也使用GICP匹配代价。
本项目的问题不能简单归为“GICP不能建图”，而应评估整套去畸变、融合、闭环和地图维护的实现及验证。
Point-LIO、FAST-LIO2本身也不等于带可靠全局闭环的完整巡检系统。

## 本农场应怎样比较

先保留现有地图和路线，给实地起点做标记。分别记录：实际停点相对标记的误差、地图上估计位置的误差、
起点附近固定路沿/树干是否重影，以及是否需要人工补目标或接管。实际走回起点和估计轨迹闭合是两项指标。

如果需要替换，优先使用同一段 L1＋IMU 原始 rosbag 对比现有方案与宇树 Point-LIO，
再验证选定闭环后端：候选是否出现、几何验证是否通过、约束是否加入、优化是否执行、历史地图是否重建。
同时记录CPU、处理延迟及资源占用。没有同数据对比和实机复测，不承诺某方案能消除现场2–3米误差。

导航中的TF延迟和速度输出不连续需要独立验收；换SLAM不会自动修好这些控制链路问题。
本次仅核查候选，没有替换现有建图、回环、定位或导航算法。
