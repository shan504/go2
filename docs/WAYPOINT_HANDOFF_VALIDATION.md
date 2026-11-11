# 多点成功接续验证

## 问题与修复范围

用户报告 Nav2 已返回成功，但剩余航点没有继续执行。旧操作桥虽然在结果回调中
调用下一点派发，却只尝试一次：TF/代价地图暂不可用就永久暂停；桥状态服务暂不可用、
查询异常或超时则清空队列。可选的到点误差诊断异常也会将已成功点放回队首。

本次在未修改的旧实现上复现：第一点返回 `action status=4 (SUCCEEDED)`，
交接时 TF 过期约 0.62 秒，路线变成 `paused=True started=False`；恢复 TF 后，
第二点仍未派发，回归断言超时。现场故障的具体瞬时原因仍需要现场日志确认。

修复位于共享 `go2_3d/operator_bridge.py`，不按地图名、目标坐标或模式分支：

- 只对已启用且未暂停的路线继续派发；watchdog 每 0.1 秒检查，暂时未就绪时约每 0.5 秒重试。
- 目标仍在队首，直到桥启用回读、定位、TF、代价地图和目标自由空间检查全部通过。
- 单次桥查询限时 2 秒；整个派发等待约 15 秒。持续故障保留队列并报告超时，需显式恢复。
- 占据/未知目标、非法网格、Nav2 拒绝或失败保持原来的暂停行为。不会假造成功或跳过点。
- 成功的动作只消费一次；可选误差诊断异常不影响成功终态或队列。
- 成功后的点间等待保留原执行授权，定位丢失仍需完整恢复证明、稳定定位、禁用确认和清图检查。
- 人工停止/取消清队列并撤销重试；超时或取消后的迟到桥回复不能派发。新路线仍需显式启用。

## 自动验证

新增 `tests/navigation_handoff_smoke.py` 使用 ROS2 Humble 的真实 action、parameter service、
TF 与地图消息。模拟端点提供 Nav2 结果和定位恢复证明；桥服务失联、迟到回复和诊断异常
由故障注入控制。测试包含：

1. 三点一次 Enable，两个成功交接分别遇到过期 TF 和过期全局代价地图，恢复后按序完成。
2. 桥服务暂不可用及查询超时自动恢复，迟到回复不重复发目标。
3. 查询中取消、重试中取消均清除尾队列，恢复依赖后仍不启动。
4. 点间定位丢失只保留未完成点；无证明、确认不足及旧证明不启动；合格证明及清图后继续。
5. 持续未就绪 15 秒后保序暂停，恢复依赖也不能越过显式恢复要求。
6. 到点诊断异常不会重走成功点，所有场景同时活跃的 action 最多一个。
7. 首次启用但从未派发任何 action 的路线，等待期间定位丢失也不能取得自动恢复授权。

执行方法（已有 Humble 和 nav2_msgs 的测试环境）：

```bash
cd /opt/go2_project
source /opt/ros/humble/setup.bash
# 如 nav2_msgs 在独立 overlay，还需 source 该 overlay/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
timeout 120 python3 tests/navigation_handoff_smoke.py
```

新回归已接入 `go2_3d/build.sh`。既有 `navigation_queue_smoke`、
`navigation_pose_route_smoke`、`navigation_collection_smoke`、`navigation_gateway_smoke`、
`automatic_resume_smoke`、`localization_pause_smoke` 用于复查队列、失败、恢复和取消协议。
上述六个既有 ROS 回归全部通过。新增交接回归在旧实现上失败，在修复后通过。
算法单测：`python3 -m unittest discover -s tests -v`，104 项通过。

验证环境使用官方 `ros:humble-ros-base` x86_64 镜像，独立编译官方 Humble nav2_msgs 接口。
这些是 ROS 协议及算法验证，没有启动 Nav2 规划器、连接 Unitree 传感器或驱动真实机器人。
地图几何、保存、坐标转换、定位判据和运动参数均未在本次修改；全地图几何单测保留通过。

## 机器人端验收

保持机器人静止，在现有导航启动后执行仓库文档的 `tools.sh restart-navigation` 更新进程。
重新选三个可达自由目标，只启用一次。每个点应出现 `SUCCEEDED` 后的下一次
`Map goal sent to Nav2 action`；最终显示 `all queued waypoints completed`。
如短时等待，检查 `Waypoint dispatch waiting` 的具体原因。确认取消后不会继续派发，
再另开一条路线检查定位暂停后的合格恢复与接续。实机结果需现场验收。
