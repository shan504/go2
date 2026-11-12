# KISS 后端验证记录

日期：2026-11-12。全部在独立云工作目录及新建测试环境执行；没有实机运动、
没有合并到 main、没有部署到机器狗。原工作目录基准为 `5b98e7b`。

## 测试层次

| 检查 | 实际结果 | 能说明什么 |
|---|---|---|
| 固定官方源码构建 | Python 3.12 和 Humble Python 3.10 均成功；真实 MapClosures、g2o、体素库检查通过 | 源码版本和接口可用 |
| 全量 Python 回归 | **201 项全部通过，25.395 秒** | 原有几何、地图、导航辅助逻辑与新后端的单元回归 |
| 后端进程异常 | 真实子进程被杀，以及受控 EOF/超时/丢失 ACK/错误提案/重复提交均被拒绝 | 异常不冒充优化成功；关闭可回收子进程 |
| 真实 ROS 后端集成 | 原 Open3D 0.14.1 mapper＋隔离 Open3D 0.19 后端通过 | TF、整图发布、保存及原定位接口兼容 |
| 真实农场同一 bag A/B | **未执行：当前云工作区没有该录制数据** | 不能判断真实农场效果改善 |
| Jetson ARM64 构建/实时性能 | **未验证**，当前锁定安装器提前拒绝该架构 | 不能称可直接在机器狗部署 |

`tests/test_kiss_backend.py` 调用真实固定版本原生算法，包含地点识别、正确与
错误几何验证、图优化、非零锚点、flush、提案重试及当前子图坐标一致性。
RPC 协议故障部分使用受控测试子进程，不能当作算法精度测试。

## 完整闭环正例：人工密集场景

通过 `tests/kiss_make_fixture.py rich` 生成刻意构造的密集箱体场景，不是连续
物理农场扫描，也不模拟 L1 的真实可见范围。

- 7 个子图；官方 MapClosures 最终检测到 132 个内点。
- 官方 ICP 几何验证接受，添加 1 条闭环约束，执行 1 次 g2o 优化。
- 全部历史点云和自由空间重建，最终导出 487,540 点 PCD 及兼容导航地图包。
- 人为注入的起末位置差由 3 m 变为约 0.429 m。

这仅证明实际原生算法及地图更新链执行成功，**不是机器狗精度或性能提升**。
该正例的测试配置（包括子图形成条件和地图分辨率）写入实际结果 JSON，不能
拿来替代默认配置或隐藏测试条件。

## 必须保留的负例：既有约 200 m 合成农场

对完全相同的 42 帧冻结 GICP 观测运行两套后端：

| 后端 | 合格候选/闭环约束 | 已应用优化 | 结果 |
|---|---:|---:|---|
| 旧 lidar 后端 | 2 条闭环约束 | 1 | 已重建地图 |
| 默认 KISS 后端 | 0 个合格候选、0 条约束 | 0 | 未修正漂移 |

另外单独试验 `voxel_size=density_map_resolution=0.25 m` 和 `0.20 m`，
均没有通过 5 内点门槛，仍无几何验证、约束或优化。没有降低门槛，没有修改
默认参数。两个分辨率必须一致，以保持官方体素重叠率验证的计数假设。

这个测试的末帧实际位于起点前方 1.5 m，报告默认将首末距离标为“间距”，
不会把它当真实闭合误差。冻结输入试验的父进程是 Open3D 0.19；报告保留
版本信息，不伪装成原 ROS Open3D 0.14 的完整前端 A/B。

实际正例、负例、匹配尺度诊断及逐阶段事件见
[kiss_backend_offline.json](validation/kiss_backend_offline.json)。它们都标记
`synthetic`，`real_world_loop_success_verified=false`。

## ROS 发布、保存、加载

`tests/kiss_mapping_smoke.py` 在新建且断网的 Humble 容器中执行，使用原系统
Open3D 0.14.1。真正的 KISS 原生库只在 Python 3.10 / Open3D 0.19 子进程中运行。

实际检查：

1. 直接提供 13 帧合成局部观测，实际产生 132 个地点识别内点、一次约束和优化。
2. 最后的部分子图由保存请求触发 flush；首次保存返回 pending，完成后才保存成功。
3. 重建、发布并保存 319,351 点地图；ROS `/map_cloud` 与重建快照逐点一致。
4. 非零地图起点保持正确；`map→odom` 只由原 mapper 发布。
5. 图优化不会刷新旧扫描的定位有效期。
6. `kiss-alpha`、`kiss-beta` 两个命名地图都经过原始/对齐保存、重复加载。
7. 原 GICP 定位在两张保存地图上实际返回有效定位，足印高度正确；定位阶段
   即使指定不存在的 KISS 解释器也能运行，证明它没有被绑到新后端。

定位 smoke 为控制测试成本，使用约 1,200 点、8 m 可见范围和 10 m 测试子图
查询范围；没有修改生产默认值，也没有放宽原时间/匹配验收门槛。
这不是密集大地图吞吐量测试，更不是农场回环实测。

## 原始 bag 与离线启动脚本

`tests/kiss_raw_bag_fixture.py` 生成明确标注为合成的静止房间原始 bag：
`/odom`、`/point_cloud2` 各 80 条、10 Hz、7.9 秒、每帧 4,100 点，无 TF/运动话题。
用原 Open3D 0.14.1 前端回放，检查保存和退出，不把这个没有环路的场景当闭环正例。

外层 `replay_kiss.sh` 实际创建了新 `--network none` 容器并完成 KISS 地图保存。
初次试跑发现 Humble 环境脚本的 nounset 兼容和后台 SIGINT 继承问题，已修正。
后续再次复制大镜像时，Docker VFS 文件系统空间不足，不能把该次运行计为通过。
清理本任务临时镜像后，在已创建且断网的独立测试容器内复测最终内部脚本；
完整保存/清理结果记录在 [kiss_backend_integration.json](validation/kiss_backend_integration.json)。

验证用镜像来自本任务新建 ROS base 容器的快照。完整 `Dockerfile.kiss-backend`
以 `go2-3d:edu` 为基础的构建尚未执行；ARM64 构建也未执行。没有操作原运行服务。
完成结果导出后已移除本任务临时 ROS 容器和快照镜像，释放磁盘；独立 worktree、
`.venv-kiss`、固定上游源码和离线结果仍保留。

## 可复现命令

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-kiss/bin/python -m unittest discover -s tests -p 'test_*.py'

.venv-kiss/bin/python tests/kiss_make_fixture.py farm /tmp/kiss-farm-journal
.venv-kiss/bin/python go2_3d/kiss_replay.py /tmp/kiss-farm-journal \
  --backend both --output /tmp/kiss-farm-trial --export-bundles

.venv-kiss/bin/python tests/kiss_make_fixture.py rich /tmp/kiss-rich-journal
.venv-kiss/bin/python go2_3d/kiss_replay.py /tmp/kiss-rich-journal \
  --backend kiss --output /tmp/kiss-rich-trial --export-bundles
```

所有输出目录必须是新目录。ROS 集成需原 Humble 依赖和新后端独立环境，
在无网络测试容器中执行
`python3 tests/kiss_mapping_smoke.py --kiss-python /opt/kiss-venv/bin/python3`；
不要把测试节点接入机器狗的 DDS 网络。

真实数据仍需提供出发、绕行、回到起点的 `/point_cloud2` 与 `/odom` 原始
录制，或者可证明坐标/时间对应的 GICP 关键帧记录。获得后用同一输入比较
候选、几何验证、约束、优化、历史地图接缝及实际起终点位置，才能判断是否
适合替换现有后端。
