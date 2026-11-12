# 可切换的 KISS-SLAM 后端（实验分支）

本分支从稳定版本 `5b98e7b8bc19d17fd8cab9f72a0cf459b73da5cc` 创建，
分支名 `feat/kiss-slam-backend`。默认仍使用 `lidar` 原后端；不自动迁移地图，
不修改巡检队列、定位门限或运动参数。本文的离线命令不启动实机运动。

这是一项后端适配，**尚未证明能改善 Go2 L1 的农场闭环**。默认 KISS 参数在
本项目既有的 200 米合成测试中未找到有效闭环，而旧后端在该测试中成功。
真实农场录制数据未在云工作区提供，因此没有真实数据上的改善结论。

## 固定的官方实现

- 原作者：[PRBonn/KISS-SLAM](https://github.com/PRBonn/kiss-slam)。
- 论文复现标签：[`IROS25`](https://github.com/PRBonn/kiss-slam/tree/db81ee31b59bc7a18af3fec8641d5867646ab57a)。
- 固定源码 SHA：`db81ee31b59bc7a18af3fec8641d5867646ab57a`，包版本 `0.0.2`。
- 实际调用官方 `VoxelMap`、`LoopCloser`、`LocalMapGraph`、`PoseGraphOptimizer`。
  地点检索使用 MapClosures 2.0.2；几何验证使用官方 Open3D 点到平面 ICP 和
  体素重叠率；位姿图使用官方 g2o 接口。
- 官方完整系统绑定 KISS-ICP，本适配**不启动 KISS-ICP**。现有 GICP 仍估计
  接受的扫描位姿；适配器将这些扫描组织为子图并传入官方后端。

源码和依赖哈希、许可证、构建方法、Jetson 限制详见
[KISS_DEPENDENCIES.md](KISS_DEPENDENCIES.md)。不是仅检查同名 PyPI 包的版本号。

## 坐标、地图及异常处理

```text
原 ROS 进程：原 Python / 原 Open3D / 原 GICP 配准与参数
  └─ 接受的 body 点云 + map 位姿
       └─ 私有子进程：固定 KISS 环境
            子图 → MapClosures → ICP 验证 → g2o 优化提案
       └─ 原地图线程：用优化位姿重放所有原始局部点云和自由空间射线
            完整重建成功 → 提交图优化 → 原子替换地图/起点/全局修正
```

- ROS 前端继续使用原解释器（Humble 验证环境为 Open3D 0.14.1）；KISS 子进程
  使用 Open3D 0.19.0。两套依赖不装到同一个 Python 环境。
- KISS 子进程不运行 ROS、不发布 TF、不发布速度。`map→odom`、
  `base_link→base_footprint` 仍由原 mapper 统一维护；`odom→base_link` 保持原来源。
- `/map_cloud`、`/registered_cloud`、`/mapping/loop_closure`、`/save_3d_map`
  保留原接口。`mapping_backend=kiss` 只实例化新后端，不同时执行旧闭环。
- 原地图分辨率、原始点云保留方式、栅格高度规则和导航地图保存流程不变。
  子图内部相对扫描位姿保持不变，图优化修正子图位姿，再重建所有历史点云；
  没有把已经融合的旧地图简单整体平移后当作优化结果。
- 保存会请求检查最后一个尚未封口的子图。检查或重建未完成时返回明确的
  pending/failure，需要等待后再次保存；不会导出旧地图冒充优化地图。
- 原生后端崩溃、RPC 超时、丢失确认和异常优化结果使后端进入失败状态，
  阻止成功保存。原 GICP 进程仍存活；不得静默切到旧图或把失败当闭环成功。
  图状态不确定时需要重启该实验会话并从记录重放。
- 官方 MapClosures 对邻近子图有排除规则，稀疏密度图也可能没有足够特征。
  原生库使用独立进程隔离，未通过放宽几何验证来强行闭环。

`kiss_*` 参数仅影响后端。初始 `kiss_submap_distance=10 m` 是针对短路线的
实验性设置（官方默认完整系统面向更大子图），未经过 L1 实测标定。
密度图分辨率默认保持 `0.5 m`；本分支未因合成测试不闭环而降低内点门槛。

## 独立工作目录与环境

已有稳定 checkout 不切分支。首次获取实验分支可在原仓库执行：

```bash
git fetch origin feat/kiss-slam-backend
git worktree add ../go2-kiss-backend -b feat/kiss-slam-backend origin/feat/kiss-slam-backend
cd ../go2-kiss-backend
```

如果该本地分支/worktree 已经存在，直接进入它，不重复创建。

Linux x86_64 的离线环境：

```bash
bash go2_3d/setup_kiss_env.sh
.venv-kiss/bin/python go2_3d/check_kiss_env.py --smoke
```

依赖安装到本 worktree 的 `.venv-kiss`，源码位于 `.kiss-deps`。不会安装到原
环境或更改系统依赖。脚本要求已有编译工具和对应 Python 开发头文件。
**这个锁定安装器当前只验证并支持 Linux x86_64；不要在 Jetson 上直接照此
宣称安装可用。** ARM64 的 Open3D 等依赖需要独立源码构建/验证，详见依赖文档。

## 已有 GICP 关键帧记录：离线对照

关键帧必须包含同时间戳的局部点云和 **GICP 位姿**，不是原生 `/odom` 配上
全局累积地图。目录格式：

```text
session.json
keyframes/<stamp_ns>.npz   # stamp_ns, points(N×3 body), pose(4×4)
```

新模式可记录接受的关键帧；记录包含后端修正及去掉该坐标跳变后的 GICP 位姿。
这不能消除 GICP 曾经使用已修正地图的反馈。因此冻结关键帧比较是后端对照，
不能冒充重新跑两次完整 SLAM。元数据会记录 `recorded_backend` 和该限制。

```bash
.venv-kiss/bin/python go2_3d/kiss_replay.py /data/SESSION \
  --backend both --output /data/TRIAL_NEW --export-bundles
```

- `--backend kiss`：仅新后端；`--backend lidar`：仅旧后端。
- `--kiss-config FILE.json`：覆盖 `KissConfig` 字段，实际配置写入报告。
- `--roi xmin ymin zmin xmax ymax zmax`：对固定结构区域计算起末扫描接缝距离，
  不先 ICP 对齐来掩盖误差。
- `--same-place-confirmed`：只在确认首末帧处于同一物理位置时使用。
  默认输出起末间距，不把不同终点的间距称为闭合误差。
- `--kiss-python /opt/kiss-venv/bin/python3`：允许旧 ROS 系统 Python 运行旧后端，
  新后端单独使用指定解释器；报告记录两边真实依赖版本。

每个后端输出 `report.json`、`events.json`、`trajectory.npz`、`map.pcd`、
`observed_free.npz` 和起点位姿。`--export-bundles` 还通过原 `save_snapshot`
生成可供原定位/导航加载的完整地图包。所有输出只写入新实验目录。

失败写入 `FAILED.json`，比较报告保持 `complete=false`；无闭环写入
`NOT_VERIFIED_no_accepted_backend_optimization`，不会伪造成功。

如果 bag 已含 `/gicp/keyframe/odom` 和 `/gicp/keyframe/cloud`，可用
`kiss_bag_extract.py --help` 查看只读提取命令；也支持明确指定的 GICP odom/scan
话题，按 header stamp 精确匹配并检查坐标系。只有原生 `/odom` 的 bag 需要下述
完整前端回放，不能直接当成 GICP 输出。

## 原始 ROS2 bag：前端和后端完整回放

先构建**新镜像**，不更新旧镜像、不启动已有服务（当前验证平台为 x86_64）：

```bash
docker build -f Dockerfile.kiss-backend -t go2-3d:kiss-backend .
```

默认基于本机已有的 `go2-3d:edu`；云环境代理 CA 参数见依赖文档。
bag 应含 `/point_cloud2`（`base_link` 局部点云）和 `/odom`（时间对应的原生里程计）。
分别启动两次独立离线回放，使用不同的新输出目录：

```bash
bash go2_3d/replay_kiss.sh /data/BAG /data/TRIAL_KISS kiss 0.5
bash go2_3d/replay_kiss.sh /data/BAG /data/TRIAL_LIDAR lidar 0.5
```

这就是离线启动/切换命令。每次新建 `--network none` 容器，bag 和代码只读挂载，
只回放 `/point_cloud2` 与 `/odom`；不回放原 TF、速度、目标或控制话题。
没有 SDK、Nav2 或运动节点，也不会停止其他容器。检查 `mapping.log` 中前端
丢帧/时间戳错误；回放成功本身不等于闭环成功。

保存后的 `data/maps/latest` 是该次实验输出，不会修改稳定目录的 `maps/latest`。
`run.json` 记录前端依赖，`dependencies.json` 记录新后端依赖，`save-result.json`
和地图元数据记录导出结果及闭环状态。

## 将来单独部署时的入口（本次没有执行）

已接入原 launch：`loop_backend:=kiss kiss_python:=/opt/kiss-venv/bin/python3`。
原 `run.sh` 也识别 `GO2_LOOP_BACKEND=kiss` 并使用新镜像，旧后端为 `lidar`。
**原 run.sh 是实机启动器，会按原有行为停止当前 go2-3d/go2-sdk 服务；
本任务仅使用上面的离线启动器，没有执行该实机入口。**

完成 ARM64 安装与真实 bag 验证、之后决定部署时，入口形式为：

```bash
GO2_LOOP_BACKEND=kiss bash go2_3d/run.sh mapping dense NEW_MAP_NAME
# 切回原建图后端：
GO2_LOOP_BACKEND=lidar bash go2_3d/run.sh mapping dense ANOTHER_NEW_MAP_NAME
```

导航/定位继续使用原 GICP 和已保存地图，选择 `lidar` 或不设置 `GO2_LOOP_BACKEND`。
不需要改动已保存巡检路线；地图几何改变后，原路线地图指纹校验仍正常生效。

## 验证及限制

详细执行记录见 [KISS_VALIDATION.md](KISS_VALIDATION.md)。

目前没有真实农场 bag、真实闭环精度、Jetson 实时性或 ARM64 镜像成功构建证据。
不得将组件合成试验的误差缩小写成农场性能提升，也不得将本分支直接视为
已验证的生产替代方案。
