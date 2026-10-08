# Go2 EDU：现有 SDK 镜像的 DDS 建图/定位/导航接入

适用当前设备：Ubuntu 20.04 ARM64 扩展电脑 192.168.123.18，Docker 镜像 go2-ros2-sdk:edu，DDS domain 0、eth0。
原 SDK、系统 ROS、机器狗固件均不修改。独立配置位于本目录 config/，地图位于 maps/。
启动脚本只会停止名为 go2-sdk 的旧开发容器。停止本次容器即可结束开发进程。
本包已做本地配置和逻辑测试；实际雷达建图、定位精度及运动响应仍需在你的设备上验证。

## 1. 上传与启动建图

Windows PowerShell（下载本 ZIP 到 Downloads 后）：

```powershell
scp "$env:USERPROFILE\Downloads\go2_edu_dds_fix.zip" unitree@192.168.123.18:/home/unitree/
```

机器人 SSH 终端：

```bash
cd ~
python3 -m zipfile -e go2_edu_dds_fix.zip .
bash ~/go2_edu_fix/run.sh mapping
```

保持启动终端打开。无需重建镜像，也无需代理下载。Windows 浏览器 Foxglove 连接：
`ws://192.168.123.18:8765`。3D 面板固定坐标设为 map，显示 /map 和 /point_cloud2。
先静置约 10 秒，应出现局部地图。用手机或遥控器缓慢绕场地走动，逐步扩展地图。
本包不转换前置相机视频，/robot0/camera/image_raw 不能用作本次建图状态判断。

另开一个 SSH 终端检查（命令只订阅数据，不发送运动）：

```bash
bash ~/go2_edu_fix/tools.sh check
```

应看到 /odom、/point_cloud2、/scan、/map，以及两段 TF。/scan 的 finite_ranges 和
/map 的 known_cells 应大于 0，最后出现 PASS。定位模式需先设置初始位置才有 map -> odom。

## 2. 保存地图

```bash
bash ~/go2_edu_fix/tools.sh save
```

保存到 ~/go2_edu_fix/maps/go2_map.yaml 及对应图像；文件保留在宿主机。
此操作保存 AMCL 使用的栅格地图，不是 SLAM 的可继续编辑位姿图。

## 3. 下次加载地图定位

在启动终端 Ctrl+C 结束建图，然后：

```bash
bash ~/go2_edu_fix/run.sh localization
```

Foxglove 3D 面板加载 /map，使用发布工具的 Pose Estimate 在地图上指定当前位置与朝向，
话题为 /initialpose（geometry_msgs/PoseWithCovarianceStamped）。在手机上小范围转动，
让激光扫描与地图墙边对齐。ROS 图中 /amcl 为 active、map -> odom 出现后再下导航目标。
如使用 RViz，等价操作是 2D Pose Estimate。

## 4. 导航

首次启动默认 enable_control=false，建图时不会发送运动请求。确认定位后：

```bash
bash ~/go2_edu_fix/tools.sh enable
```

使用 ROS 2 官方 action 命令发送目标，另开 SSH 执行：

```bash
sudo docker exec -it go2-sdk /bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

上面的 x、y 只是地图坐标格式示例，必须改为地图上已观察到的空地目标；第一段路径选近处。
RViz 的 Nav2 Goal 也是同一 action。仅发布 /goal_pose 不保证 Nav2 收到导航 action。

关闭运动接入（有正在发出的运动时会发送 StopMove）：

```bash
bash ~/go2_edu_fix/tools.sh disable
```

首次运动需要确认机器人处于可行走状态。适配器不会自动起立、切换控制模式或接管电机低层。
运动使用 Unitree 官方 /api/sport/request，Move API 1008、StopMove API 1003。
速度限制 0.15 m/s、角速度 0.3 rad/s；命令超过 0.5 秒不更新，或雷达/里程计超过 1 秒
不更新，会发送 StopMove。正式导航前仍需实机验证这个接口确实响应。

## 处理内容

- 原生 /utlidar/cloud_base -> /point_cloud2 -> /scan -> SLAM /map。
- 原生 /utlidar/robot_odom -> /odom 与动态 odom -> base_link TF，保留真实三维姿态。
- 雷达与里程计使用同一个时间偏移，保留传感器间相对时间，不修改主机/控制器时钟。
- 在 URDF 副本中去除固定 map -> odom 和 odom -> base_link，交由 SLAM/AMCL 与里程计发布。
- Nav2 配置副本全部使用真实时钟，初始全局代价地图 60×60 m，收到地图后由静态层更新范围。
- Foxglove 只公布需要的标准话题，避免缺少机械臂/厂商接口包的日志及无用大消息。

原项目：https://github.com/abizovnuralem/go2_ros2_sdk
官方运动接口：https://github.com/unitreerobotics/unitree_ros2/tree/master/example/src
