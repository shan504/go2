# Go2 EDU：三维建图、GICP 定位、Nav2、Foxglove

目标设备：Ubuntu 20.04 ARM64 扩展电脑 `192.168.123.18`，已有
`go2-ros2-sdk:edu`（容器内 ROS2 Humble），CycloneDDS 使用 `eth0`、domain 0。
宿主机 Foxy/Noetic 不变。原始 ZIP 和解压后的 `patches/` 保留供追溯，
**本次使用 `go2_3d/`，不要启动原包的二维 SLAM/AMCL 入口。**

## 先在扩展电脑安装并启动

首次安装，在 SSH 终端执行（独立目录，保留原 `~/go2_ros2_sdk`）：

```bash
git clone https://github.com/shan504/go2.git ~/go2_nav
cd ~/go2_nav
bash go2_3d/build.sh
bash go2_3d/run.sh mapping
```

已有 `~/go2_nav` 时先执行 `git -C ~/go2_nav pull --ff-only`。
构建需要外网下载 Ubuntu/ROS 软件包；脚本基于现有 ARM64 SDK 镜像，
不重编 SDK，不要求 Docker Compose。构建后自动执行算法测试和 ROS 模拟传感器测试。
`run.sh` 保持前台运行，会停止名为 `go2-sdk` 的旧开发容器以避免重复 TF，保留其容器文件。

## 三维地图与定位链路

```text
/utlidar/cloud_base ── DDS bridge ── /point_cloud2 (base_link)
/utlidar/robot_odom ── DDS bridge ── /odom + odom → base_link
                                     │
                        3D GICP scan-to-map
                                     │
                      /map_cloud + map → odom
                                     │
            保存 map.pcd + 同坐标系 nav.pgm/nav.yaml
                                     │
             加载 map.pcd + /initialpose → GICP 定位
                                     │
             Nav2 + 实时 /scan → /cmd_vel → Sport API
```

使用 **Open3D Generalized ICP**，不是 AMCL 或普通点到点 ICP。
地图由三维扫描与连续里程计的预测姿态配准后累积，首帧机身位置是 `map` 原点。
只加入通过重叠率、残差及位姿修正阈值检查的关键帧；保存二进制 PCD。
这是增量三维 scan-to-map 建图，**没有闭环优化**，较大场地可能累积漂移。
原生点云的运动畸变补偿、里程计质量仍取决于机器狗输出。

定位加载三维 PCD；在已知大致位置发布 `/initialpose` 后做 GICP 精配准。
重定位同样重新设置初始位姿；不提供无初始位置的自动全局搜索。
`map → odom = map → base_link × inverse(odom → base_link)`，不会同时启动 AMCL 或 slam_toolbox。

## 在 Foxglove 操作

使用你现有的 Foxglove 连接，端口保持 8765。面板和发布操作见
[Foxglove 操作说明](docs/FOXGLOVE.md)。

- 原始桥接点云：`/point_cloud2`；GICP 配准点云：`/registered_cloud`；三维地图：`/map_cloud`。
- 固定/显示参考系均为 `map`，网格 `Z 向上`；配准点云与地图按 `z` 高度着色。
- 前置相机：`/camera/image/compressed`，替换旧 `/robot0/camera/image_raw`。
- 发布 `/mapping/save` 保存三维地图和 Nav2 栅格。
- 设置 `/initialpose`；点选 `/goal_pose` 会转换成 Nav2 `NavigateToPose` action。
- 发布 `/control/enable` 启用或停止运动；`/navigation/cancel` 取消导航。
- 查看 `/localization/status`、`/localization/valid`、`/operator/status`。

地图保存到仓库目录的 `maps/时间戳/`，`maps/latest` 只在全部保存成功后更新。
GICP 使用 15 厘米降采样，显示/PCD 地图使用 6 厘米，保留三维高度；
静止时每隔 2 秒将通过匹配检查的扫描加入地图，补充单帧雷达覆盖。
地面默认在 `map` 坐标系 `z=-0.30 m`；建图前测量地面并调整
`go2_3d/gicp.yaml` 的 `floor_z` 和障碍高度切片。栅格只标记观测到的自由空间，
未观测区域保持未知，三维障碍投影覆盖自由空间；Nav2 不允许穿过未知区域。

保存后，在启动终端 Ctrl+C，再启动：

```bash
# 只定位，不运行 Nav2
bash go2_3d/run.sh localization
# 或定位 + Nav2，运动仍默认关闭
bash go2_3d/run.sh navigation
```

**首次启动和模式切换目前通过 SSH；显示、保存、定位初始化、导航目标与运动开关通过 Foxglove。**

## 运动与实机验收

只有导航模式、GICP 匹配有效且明确启用运动时才接收控制。
速度上限 `0.15 m/s`，角速度上限 `0.3 rad/s`；里程计/雷达超过 1 秒未更新、
GICP 无效或超过 1 秒未完成匹配、运动命令超过 0.5 秒未更新时发送 StopMove。
定位失效还会请求取消 Nav2 目标。机器人不会自动起立或切换运动控制模式。
上述是软件保护；最终需要现场验证官方 Sport Move/StopMove 接口。

正式导航前确认：

1. 相机持续出图，点云为真实三维数据；`odom → base_link` 连续，只有一个 `map → odom` 发布者。
2. 小场地慢速手动建图，PCD 重载正确，栅格中的地面、墙体与通道正确；不使用仅显示累积的点云作为地图。
3. GICP 连续有效，三维扫描与地图对齐；初始位姿偏离时不会误报定位成功。
4. 测量并调整 `go2_3d/prepare.py` 中含腿足的 footprint，确认 Nav2 规划不穿障碍。
5. 在空地验证小幅运动与停止，随后发送近距离目标；定位/传感器失效时应停止。

## 已验证与尚待验证

云端 x86_64：7 项真实 GICP/几何/PCD/栅格测试，使用 Open3D 0.19 和 Ubuntu
Jammy 的 0.14 均通过；ROS2 Humble 模拟三维传感器测试通过建图、TF、Foxglove 保存接口、
地图重载、初始位姿定位与传感器丢失失效检查。原 DDS 包 4 项测试及 SDK 配置生成检查通过。
Foxglove 导航接口通过真实 `NavigateToPose` 测试 action server 验证：无定位时拒绝操作、
点选位姿转 action、启用/关闭运动参数、定位丢失时取消目标。该测试不执行 Nav2 路径规划或运动。
SDK 配置检查使用上游提交 `b440609591a249e7bdd4bbc88e056a3660575447`。

**尚未在 ARM64 实机运行新镜像，尚未验收相机固件响应、Nav2 规划、定位精度或真实运动。**
云端完整 Nav2 镜像构建被 ROS 软件源网络访问阻止；模拟测试镜像使用已有 ROS2 基础依赖
及签名验证的 Ubuntu Open3D 软件包。不能把这些测试等同于机器狗已完成导航。

本地算法测试：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -v
```

依赖：`numpy`、`PyYAML`、`open3d`。`tests/ros_smoke.py` 另需 ROS2 Humble。
