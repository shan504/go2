# 有限导航恢复与停车排查

`navigate.xml` 在整个 `PipelineSequence` 外使用 Humble 官方 `RecoveryNode`。
初次执行失败后最多重试两次，每次通过官方 `Wait` 等待 2 秒，仍使用同一个
`NavigateToPose` 目标。每个目标拥有独立的次数预算；正常完成后才推进下一个航点。
这也会有限重试 FollowPath 返回的失败，不按未提供错误码的 Humble action 猜测失败原因。

必须包住整个序列：Humble 的 `PipelineSequence` 在任一子节点返回 FAILURE 时
先 halt 所有子节点。于是重规划失败会先调用 FollowPath 的 action cancel，再进入
Wait；不能只包装 ComputePathToPose，让旧控制器在规划恢复中继续跑。
FollowPath 的 `server_timeout=1000` 单位为毫秒，给 action 响应、取消和终态结果
留出时间，避免默认 20ms 比 10Hz 控制器的取消处理周期更短。这不是 TF 容差，
也不改变控制频率、速度、进度判定或 GICP。控制器不响应取消属于额外故障，
这个 XML 不能代替失联保护或底盘急停。

恢复不清空代价地图，不旋转、不倒退，不允许未知空间，不跳过碰撞检查。
人工停止/取消、定位暂停通过已有 Operator 取消 NavigateToPose，整棵树连同 Wait
一起 halt。恢复本身不启用运动桥。定位丢失后的恢复授权仍由原有 Operator 规则管理，
并未由规划重试额外授权。持续失败耗尽预算后返回 ABORTED，原队列保留失败目标
和后续目标并暂停；`tools.sh resume` 仍只是人工重试，不是绕过不可达检查。

`prepare.py` 对不同 SDK 预设补齐上述 BT 库和 Wait behavior，保留其他插件。
所有地图、standard/dense、保存路线和点选路线使用同一棵树。
普通 lidar 启动不再传入空的 `camera_calibration:=` 或 `session_directory:=`；
非空 RTAB 参数仍原样传递，不生成标定数据。

## 本次现场证据与范围

2026-11-13 的日志先出现 GridBased 重规划失败，然后 BT 中止 FollowPath，最后
Operator 收到 ABORTED 并保留 13 点。首点距目标仍约 23m，不能解释为已到达后未
派发第二点，也没有证据将它归为进度超时。几分钟后的里程计超时不是最初触发原因。
有限恢复只解决暂时失败立即终止整个任务的行为；尚未取得失败瞬间的全局代价地图，
无法判定最初无路是起点、目标、狭窄通道、动态障碍还是地图更新造成。

已有只读检查可在停车后执行（不发送目标或运动命令）：

```bash
bash go2_3d/tools.sh navcheck 24.45 5.44
```

它报告起点/目标栅格代价、连通性、静态净空、动态代价地图、TF 和生命周期。
停车后的检查不是失败瞬间快照；若恢复后地图已改变，不应据此反推唯一根因。
保存的第 12 点 (-0.34, -10.09) 当时为障碍值 100，第 13 点 (-1.45, -0.01)
为未知值 -1。保存允许记录这些点，但执行时仍会拒绝并保留暂停，须核对地图及点位。

Humble RotationShimController 的 `getSampledPathPt()` 在新路径上将采样点时间设为
`clock_->now()`，随后 `transformPoseToBaseFrame()` 查询到 base frame 的变换。
这能解释为何每秒新路径附近会出现比最新 map→base_footprint 晚几十毫秒的请求，
但还不能证明机器人安装版本和每一条报错都来自这条路径。此次没有篡改时间戳、
修改上游控制器或扩大 TF 容差，也不将它当作 GridBased 无路的已证实原因。

## 离线验证

`tests/navigation_retry_smoke.py` 使用真实 Humble BT navigator、官方 BT 插件和
Wait behavior，ComputePathToPose/FollowPath 使用隔离的合成 action server，
并接入真实 Operator。测试覆盖三个目标、暂时/持续规划失败、150ms 延迟取消、
取消后的静止等待、人工恢复、停止/取消和定位暂停。合成运动记录不发布到底盘话题。
`tests/navigation_grid_start_smoke.py` 执行实际启动脚本的 CLI 组装，验证空值省略和
带空格非空参数保留。`build.sh` 包含新增恢复测试。

这些检查不运行真实 GridBased/DWB，不含农场地图、ARM64 部署或机器人运动验证。
实机是否不再两米停车，仍需部署后确认；持续无路仍必须保留失败。

本轮云端 x86_64 实际结果：160 项单元测试、6 项 RTAB launch 测试通过；
启动 CLI、三模式 launch、真实 SDK 源码生成配置检查通过；现有队列、离线路线、
定位自动恢复授权测试通过；新增真实 BT/Wait 恢复测试通过，停止并重新启动验证容器后再次通过。
ROS 依赖基于官方 Nav2 Humble 提交
`3c3db59d6969d8ecee8e68468693d006397f4a0c` 编译的 11 个包，
SDK 配置检查使用上游源码 `b440609591a249e7bdd4bbc88e056a3660575447`。
完整 `go2_3d/build.sh` 需要机器人已有的 `go2-ros2-sdk:edu` 镜像，本云机未运行该整套实机镜像构建流程。
