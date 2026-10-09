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
地图由三维扫描与连续里程计的预测姿态配准后累积，首帧机身位置、朝向是 `map` 原点。
新建图保留里程计 Z 轴方向，不再把首帧机身的 roll/pitch 当成地图轴方向。
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

可导入 [预设 Foxglove 导航布局](docs/foxglove_go2_navigation.json)，左 3D 面板用于初始位姿，
右 3D 面板用于目标点。场景右侧尺子下方的发布工具需要先启用，再依次**单击位置、单击朝向**；
这不是 RViz 的拖动后松开操作。详见上面的操作说明，布局导入兼容性尚待用户桌面确认。

地图保存到仓库目录的 `maps/时间戳/`，`maps/latest` 只在全部保存成功后更新。
建图 GICP 使用 15 厘米降采样，定位使用 5 厘米细配准，标准显示/PCD 地图使用 6 厘米，保留三维高度；
静止时每隔 2 秒将通过匹配检查的扫描加入地图，补充单帧雷达覆盖。
需要更稠密的室内地图时，使用 `bash go2_3d/run.sh mapping dense`：保存分辨率为
2 厘米，上限 100 万点，将最近 0.5 秒扫描按各自时间戳的里程计对齐后配准。
GICP 仍使用 15 厘米点云及缓存的粗地图；全图每 3 秒发布一次，Foxglove 发送缓冲为 32 MiB。
2 厘米是保留点的分辨率，不代表定位精度，也不会补造雷达未扫描到的表面。
该窗口没有实现单个扫描包内部的逐点 IMU 去畸变。参数增加后若出现处理超时，应降低点数或使用标准配置。
静止里程计下单次 GICP 校正跳变超过 5 厘米或约 2° 时会拒绝匹配并使定位无效；
明确重新发布初始位姿可以建立新的校正。
保存时从起点附近、机身下方的点云估计地面平面，支持初始机身姿态造成的小幅倾斜。
`floor_z=-0.30` 是搜索先验，障碍按距拟合地面的高度判断；保存 `ground_model` 和
`observed_free.npz`。观测到的地面端点补充自由单元，真实障碍覆盖自由空间，未观测区域保持未知。
实时 Nav2 障碍也按同一个地面平面过滤，不再使用固定的机身 z 切片。
估计不到有足够支撑的地面时拒绝保存/修复，不能把任意平面或未知区域当作空地。

保存后，在启动终端 Ctrl+C，再启动：

```bash
# 只定位，不运行 Nav2
bash go2_3d/run.sh localization
# 或定位 + Nav2，运动仍默认关闭
bash go2_3d/run.sh navigation
# 使用稠密扫描窗口定位 + Nav2
bash go2_3d/run.sh navigation dense
# 若实际回到建图起点、同一朝向和高度，可在启动命令中给初始位姿
bash go2_3d/run.sh navigation dense 0 0 0
```

数值参数为 `X Y YAW_DEGREES [Z]`，单位米、度；只提供提示，不会跳过 GICP 接受检查。
Nav2 生命周期启动等待 GICP 连续有效和新鲜 `map → base_footprint`，不再提前激活并反复等待不存在的 map。
`/navigation/startup_status` 显示等待、启动或失败原因。默认运动仍关闭。

旧图需要修复导航投影，先停止正在运行的导航终端，再运行：

```bash
bash go2_3d/tools.sh repair-map
```

该命令离线读取 `maps/latest/map.pcd`、估计地面、重新生成导航栅格，成功后更新 `latest`。
原图目录保留，新目录的 PCD 与原文件完全相同；输出旧/新栅格自由、障碍、未知单元数量。
它修正导航投影，不修复 PCD 自身的漂移、重影或缺失扫描；这类问题需要原始传感器记录或重新建图。

### 在宽阔场地重新建图并统一地面坐标

已有导航终端先按 Ctrl+C 结束。将机器狗放在宽阔场地的起点，记住位置和朝向，执行：

```bash
cd ~/go2_nav
git pull --ff-only
bash go2_3d/run.sh mapping dense
```

保持该终端运行，用遥控器慢速绕场地建图，再回到起点、同一朝向并保持站立。
在第二个 SSH 终端保存，确认输出 `success=True`：

```bash
bash ~/go2_nav/go2_3d/tools.sh save
```

回到启动终端按 Ctrl+C，再执行：

```bash
cd ~/go2_nav
bash go2_3d/tools.sh level-map
bash go2_3d/run.sh navigation dense 0 0 0
```

`level-map` 保留原目录，将**整张三维 PCD**做刚体坐标变换，使拟合地面水平且 `z=0`，
同时在相同坐标系生成 5 厘米 Nav2 栅格。所有三维点保留，不缩小机器狗外形或删除障碍。
新图保存变换矩阵和站立机身高度；`latest` 只在全部写入成功后更新，重复执行不会再移动原点。
这是离线处理，必须停止加载旧图的容器；代码通过目录挂载生效，不需要重建镜像。
此后 Foxglove 二维 `/initialpose` 的 `z=0` 会自动补入保存的站立机身高度，
显式非零 Z 仍表示机身高度。必须重新选择新图里的目标，不能沿用旧图的目标消息。
这项变换修正地面坐标，不能代替闭环优化或修复配准漂移。

保持导航终端运行；Foxglove 固定和显示参考系都选 `map`，关闭跟随和同步时间戳。
确认 `/localization/valid` 持续为 true、`/navigation/startup_status` 显示 Nav2 启动成功，
然后在第二个 SSH 终端执行 `bash ~/go2_nav/go2_3d/tools.sh enable`。
启用仅允许接收速度，**不会自行前进**；还需要在 Foxglove 发布一个新图自由区域的近距离目标。
先选约半米远的目标，观察 `/plan`、`/operator/status` 和实际运动。

现有图仍异常时，运行 `bash go2_3d/tools.sh export-map`，导出
`runtime/go2-map-debug.zip`（当前 PCD、栅格、地面模型、生成配置和最近容器日志）。
这是只读打包，不会修改地图或发送运动命令。公开 Go2 项目对照及此次修正依据见
[Go2 实现对照](docs/GO2_REFERENCE_REVIEW.md)。

Nav2 使用动态 `base_footprint`：投影到 map 栅格的 XY 平面，保留实际位置和 yaw。
真实 `base_link`、里程计和三维点云仍保留六自由度姿态。机身倾斜不再倾斜全局导航轮廓；
真实约 6° 的朝向偏差仍应显示。局部栅格仍在 odom 中，map/odom 的真实三维倾斜仍可能在显示中体现。
`/goal_pose` 仅经操作节点处理，Nav2 原生的同名目标订阅已隔离。
目标若位于地图障碍、未知空间或代价地图净空不足处，会在 `/operator/status` 明确拒绝。
已验证的目标以固定 map 位姿发送，不携带历史机身时间；规划失败直接结束，不自动转圈/倒退。

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

云端 x86_64：31 项真实 GICP/几何/PCD/地面/栅格/去噪/静止跳变及校正滤波测试，使用 Open3D 0.19 和 Ubuntu
Jammy 的 0.14 均通过；ROS2 Humble 模拟三维传感器测试通过建图、TF、Foxglove 保存接口、
地图重载、初始位姿定位与传感器丢失失效检查。原 DDS 包 4 项测试及 SDK 配置生成检查通过。
定位新增 5 厘米细 GICP 和仅针对已接受校正的滤波；模拟测试确认错误参考系不会初始化，
静止跳变被拒绝后不会更新 TF 或报告有效，传感器失效保护继续生效。
Foxglove 导航接口通过真实 `NavigateToPose` 测试 action server 验证：无定位时拒绝操作、
点选位姿转 action、启用/关闭运动参数、定位丢失时取消目标。该测试不执行 Nav2 路径规划或运动。
SDK 配置检查使用上游提交 `b440609591a249e7bdd4bbc88e056a3660575447`。

稠密配置另经 ROS 三维传感器建图/定位测试，100 万点 XYZ 消息通过实际 ROS CDR 序列化往返。
用户已在 ARM64 镜像中通过原版算法与 ROS 接口测试，实机诊断确认相机、三维雷达、地图及 GICP TF 有数据。
用户已确认稠密配置下 GICP 曾连续有效，Nav2 控制器与规划器为 active；随后静止匹配仍有间歇拒绝。
用户后续实机诊断确认细 GICP 有效，fitness=1.000、RMSE=0.025m；失败目标在全局和局部
代价地图均为 99，静态地图为自由单元。RMSE 是匹配残差，不代表真实定位误差。
**导航路径与真实运动仍待现场验收。**
局部地图为 4×4 米、5 厘米分辨率；按用户要求，两张地图总膨胀半径现为 0.05 米，
衰减系数 12，控制器继续使用真实外形碰撞检查。这是总半径，不再额外加上 0.23 米。
接受的校正在有效期内按真实里程计时间最多 20 Hz 发布 TF；ROS 测试验证匹配间持续更新，
拒绝匹配/数据过期后停止刷新。Humble MessageFilter 测试复现延迟 TF 的激光超时丢弃，
连续 TF 下测试扫描全部通过；该修复还需实机检查。
云端完整 Nav2 镜像构建被 ROS 软件源网络访问阻止；模拟测试镜像使用已有 ROS2 基础依赖
及签名验证的 Ubuntu Open3D 软件包。不能把这些测试等同于机器狗已完成导航。

地面修复新增 4 项测试：倾斜地面不再成为障碍、真实障碍保留、旧图修复保持原 PCD/旧目录，
无地面时不更新 latest、随机体积点云不当作地面；Open3D 0.19 和 0.14 均通过。
ROS 测试验证数值初始位姿走真实 GICP，Nav2 就绪节点在有效标志和 TF 同时满足前不发启动请求，
满足后仅发一次；模拟机身侧倾时实时过滤移除地面、保留障碍，定位失效时停止输出。
这不代替用户这张实际地图的修复结果或真实 Nav2 路径验收。

2026-10-09 读取用户上传的真实 163426 点地图，验证地面刚体对齐、原图不变、所有点保留，
导航栅格从 10 厘米改为 5 厘米。使用该 PCD 采样的合成扫描在真实 ROS2 Humble 中验证：
二维初始位姿 z=0 自动补入站立高度，GICP 输出机身 z≈0.385 米、导航轮廓 z=0。
这验证了坐标和初始化契约，**不是实机传感器回放，也不是实际路径/运动验证**。
新场地地图按上面的建图、保存、level-map、定位流程现场验收。

细栅格转换已修正：旧自由单元按已知面积转移，不再只转移中心子格；未知/障碍不当作自由区域。
旧 `level-map` 图可在停止容器后执行 `tools.sh repair-map`，从保留的前一张粗栅格恢复自由区域，
当前 PCD 字节和定位坐标保持不变。稀疏地面回归由旧版的 200 个自由子格恢复为 800 个；
直接编译未修改的 Humble NavFn A* 核心复现旧图无路径、验证修复后有路径，保留 0.23 米内切净空。
此测试不运行完整 planner_server、控制器或真实运动。`navcheck` 另比较静态内切净空与实时全局地图，
区分地图/净空断路和额外实时障碍标记断路。目标中心为自由格不代表整条路线可达。

## 独立小障碍过滤

全局顺序为 `static_layer → obstacle_layer → denoise_layer → inflation_layer`，
局部顺序为 `obstacle_layer → denoise_layer → inflation_layer`。
使用 Humble Nav2 官方 `DenoiseLayer`，在膨胀前删除独立的 1～7 格障碍；
八方向相邻格算同一块，8 格及以上的连通障碍保留。移除的小块不会再产生膨胀区。
这按占据格大小过滤，不能识别物体；真实小石块、细杆等也可能被忽略。
农场地形的坑洞、台阶、坡度可通行性不由这个二维去噪层判断。

参数统一放在 [navigation_filter.yaml](go2_3d/navigation_filter.yaml)：
默认 `minimal_group_size: 8` 表示删除 1～7 格；设为 3 时仅删除 1～2 格。
改变参数后清理栅格并重启导航，两张代价地图使用同一阈值。
格数对应的实际面积取决于地图分辨率：在 5 厘米栅格中，7 格面积为 0.0175 平方米。

## 当前容器直接切换到 5 厘米膨胀

导航保持运行，先通过 Foxglove 停止运动。宿主机执行：

```bash
cd ~/go2_nav
git pull --ff-only
bash go2_3d/tools.sh inflation-5cm
```

两行 `actual inflation_radius=0.050m` 来自正在运行的全局、局部节点的参数回读。
该命令只修改两项膨胀参数，触发 Nav2 重算，不发送目标或运动，不需要重建镜像/地图。
生成配置也已改为 0.05，之后重启会保持相同参数；当前旧 `runtime/config` 显示的是启动时配置，
即时参数以回读为准。当前 5 厘米地图外扩一格；粗于 5 厘米的旧图会受栅格量化影响。

0.05 米小于当前外形的 0.23 米内切半径，Nav2 可能输出对应警告；这是本次明确选择的参数。
全局规划的距离缓冲因此减少，DWB 的 `ObstacleFootprint` 仍检查真实外形；一条全局路径
不能据此保证控制器可执行。当前实机日志已到达控制器，随后报 `Failed to make progress`。
先启用运动，再发送新目标，避免运动禁用时也消耗进展检查的 10 秒时间。

`tools.sh navcheck X Y` 新增 `/cmd_vel_nav`、`/cmd_vel` 消息数/非零数/速度峰值及里程计位移。
执行目标期间读取这些值，可区分控制器没有输出、速度平滑器链路没有输出，和命令已输出但
机器狗未移动。该检查仍不发送任何目标或运动。实时参数工具通过真实 ROS 参数服务测试，
包括回读 0.05 和参数拒绝处理；不是完整实机运动验证。

## 清理现有静态地图

已有导航先发布停止运动，再在启动终端按 Ctrl+C。在宿主机执行：

```bash
cd ~/go2_nav
git pull --ff-only
bash go2_3d/tools.sh clean-map
bash go2_3d/run.sh navigation dense
```

`clean-map` 备份为一个新目录，仅修改导航 PGM；原 PCD、地图坐标、站立高度及原目录保留，
不用再次 `level-map`。它也让 `/map` 和操作节点的目标检查看到清理结果。
新建地图保存、地面对齐、栅格修复时自动应用相同静态过滤。
启动会先检查当前镜像是否含 `DenoiseLayer`；若提示缺失，执行 `bash go2_3d/build.sh` 更新镜像。
重新初始化 GICP、确认定位有效，再启用运动并**重新发送目标**；旧失败目标不会因启用按钮而重试。

验证直接编译上游未修改的 Humble 去噪算法及 NavFn 核心：同一 0.6 米通道，
去噪前 0 步路径，去噪后 98 步；连续墙体与更大障碍仍阻断路径，未知格保持原样。
上传的旧 163426 点地图另删除 37 个小连通块、59 个占据格，PCD 的 SHA256 不变。
这些是算法、配置与 ROS 接口验证，尚未在当前 153401 点新地图上运行完整 Nav2 或真实运动。
源码及研究记录见 [GO2_REFERENCE_REVIEW.md](docs/GO2_REFERENCE_REVIEW.md)。

去噪规划回归（需要 Humble ROS 开发环境及完整上游 Humble Nav2 源码）：

```bash
cmake -S tests/navfn -B /tmp/go2-denoise-build -DGO2_NAVFN_SOURCE=/path/to/navigation2/nav2_navfn_planner
cmake --build /tmp/go2-denoise-build -j2
python3 tests/denoise_navfn_smoke.py /tmp/go2-denoise-build/grid_plan
```

本地算法测试：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -v
```

依赖：`numpy`、`PyYAML`、`open3d`。`tests/ros_smoke.py` 另需 ROS2 Humble。

ROS2 Humble 环境中的只读导航诊断测试：`python3 tests/navcheck_smoke.py`。
TF 过滤回归需 Humble 的 rclcpp、tf2_ros、sensor_msgs 和 message_filters 开发包：

```bash
cmake -S tests/tf_filter -B /tmp/go2-tf-filter-build
cmake --build /tmp/go2-tf-filter-build -j2
/tmp/go2-tf-filter-build/filter_check
```

细栅格断路的原生 NavFn 回归（需未修改的 Humble `nav2_navfn_planner` 源目录及 rclcpp）：

```bash
cmake -S tests/navfn -B /tmp/go2-navfn-build -DGO2_NAVFN_SOURCE=/path/to/navigation2/nav2_navfn_planner
cmake --build /tmp/go2-navfn-build -j2
python3 tests/navfn_smoke.py /tmp/go2-navfn-build/grid_plan
```
