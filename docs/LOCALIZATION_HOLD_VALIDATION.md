# 公共定位容差与受限里程计保持

本次调整适用于所有所选地图的公共定位流程。现场第一点约5.4秒后取消的旧日志
不能区分配准拒绝、传感器延迟和定位心跳中断；本次没有据此认定根因。

`go2_3d/gicp.yaml` 与 Mapper 默认值同步：对应距离0.70m、最低fitness 0.20、
最大RMSE 0.20m，运动创新限幅1m/45°，静止校正限幅0.35m/12°。
这些是软件接受阈值，不代表定位精度。建图参数、地图密度、速度和平滑系数不变。

保持沿用上次接受的 `map -> odom` 校正，以真实、带时间戳的里程计推进机身和footprint。
它从上次合格扫描的原始接收时间起，最多3秒、累计行程1m、累计转角60°。
路程和转角按新里程计样本累计；折返和反向旋转不能补回预算，预算超限会锁住保持，
只有真正新鲜的合格配准能重置它。重复TF不重复累计。

保持必须持续具备有效点云（接收/时间戳年龄≤1秒）和里程计TF（年龄≤0.3秒），
并保留已有最多100ms的未来时间容差。首次启动或显式重定位后不能用里程计假装配准成功。
低质量、缺点、旧结果均不能更新合格校正或时间；空包和扫描窗口中的旧几何不能续期。
非有限结果、超出创新上限或静止跳变上限的结果直接撤销有效性，不使用保持掩盖异常跳变。

`/localization/state` 以10Hz报告 `tracking`、`odometry_hold`、`registration_invalid`、
`registration_timeout`、`cloud_timeout`、`cloud_timestamp_stale`、`odometry_timeout`、
`odometry_unavailable`、`odometry_clock_regressed`、`odometry_distance_limit` 或
`odometry_angle_limit`，附合格年龄、里程计预算和最近匹配失败原因。
Mapper在状态切换及每2秒输出日志。操作层暂停和CANCELED结果还区分：

- `localization_reported_invalid`：收到的有效性消息为false；附最近Mapper状态和状态年龄。
- `localization_heartbeat_timeout`：0.5秒没有新有效性消息；不能据此断言GICP配准失败。

两个话题的回调没有原子顺序，最近Mapper状态可能滞后，因此日志明确记录它的接收年龄。
只读 `tools.sh motioncheck` 同时收集这些状态和公共参数。暂停仍保留全部航点；
恢复后必须显式启用，只有新速度命令可以进入底盘桥，不回放失效期间的命令。

## 云端验证

离线使用Python 3.12、Open3D 0.19.0；真实ROS使用x86_64 Ubuntu 22.04、
ROS2 Humble、Ubuntu Open3D 0.14.1，均设置 `OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1`。
基础镜像是官方 `ros:humble-ros-base`，摘要
`sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19`。
ROS软件包源被云网络策略阻止后，使用镜像内ROS及官方源码编译Nav2消息和Unitree消息，
没有关闭包签名、校验和或TLS验证。外部源码未修改：

- ros-navigation/navigation2，Humble：`3c3db59d6969d8ecee8e68468693d006397f4a0c`，编译nav2_common/nav2_msgs。
- ros2/rosidl_dds，Humble：`ab8497770c652edb40d6b1591118198cbcf14237`，编译rosidl_generator_dds_idl。
- unitreerobotics/unitree_ros2：`668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`，编译unitree_api。
- abizovnuralem/go2_ros2_sdk：`b440609591a249e7bdd4bbc88e056a3660575447`，只读取URDF/配置验证公共预检。

可在已有Humble SDK环境中复现关键检查：

```bash
cd /opt/go2_project
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
python3 -m unittest discover -s tests -v
python3 tests/localization_patrol_smoke.py
python3 tests/localization_hold_smoke.py
python3 tests/localization_pause_smoke.py
python3 tests/localized_control_smoke.py
```

在不同ROS_DOMAIN_ID中运行独立回归，避免合成传感器互相干扰。本次通过：

- 55项离线单元测试；包含真实GICP约23%重叠的新旧门限对照，无重叠仍拒绝。
- 真实ROS/GICP的28cm/10°校正、40cm/15°静止跳变拒绝，及新鲜配准恢复。
- 拒绝扫描期间真实里程计TF推进、原始3秒期限、累计距离/转角限制、传感器超时和重定位撤销。
- 四航点在配准失效及心跳中断时全部保留，恢复不自动启动；真实ROS action的排队、取消、显式恢复。
- 标准/稠密建图、PCD重载、地面/机身/footprint坐标、多个原始/对齐地图和重复加载。
- 所有mapping/localization/navigation及standard/dense的真实ROS launch参数生成。
- 自动保存起点初始化、90°/180°转弯、静止滤波参考、云先于TF到达。
- Nav2启动门的轻微未来TF及混合时钟偏差检查；`456c59c` 修复保留。
- 底盘桥新鲜命令要求、无效定位/心跳/点云/非有限速度停止、只读诊断。

这些使用真实ROS、TF和Nav2 action接口及合成端点，未运行完整机器狗SDK镜像构建、
完整Nav2规划控制闭环或真实机器狗。农场稀疏几何下的误匹配率、里程计漂移、Jetson延迟、
实际停止/恢复及路线成功率仍需实机确认，不能用合成测试替代。

## 更新启动

停止旧导航，把狗放在所选地图保存的原建图起点和朝向后：

```bash
cd ~/go2_nav
git pull --ff-only
bash go2_3d/run.sh navigation dense
```

本次不增加镜像依赖，现有镜像已具备Nav2插件时无需重建。定位有效后收集航点，再显式启用。
若当前导航仍运行且要保留当前位置，可在第二终端执行 `bash go2_3d/tools.sh restart-navigation`；
该命令先禁用运动、清理旧目标并验证新鲜位姿，按既有行为重启，随后重新选点和启用。
