# 多点连续派发

修复范围是 `operator_bridge.py` 的点位接收、结果处理和下一点派发，适用于所有地图。
建图、GICP、TF生成、Nav2规划/控制、到点参数及地图文件均不改变。
当前源码到点条件仍为平面距离30厘米内、终点朝向不限。只有Nav2返回
`SUCCEEDED`才增加完成数量；`ABORTED`、拒绝和异常都不能冒充成功跳点。

## 派发规则

- 新路线先收集，启用一次后按编号串行执行，始终至多一个活动action。
- 成功结果提交完成状态，由已有100毫秒watchdog继续推进下一点。
- TF、代价地图或action暂不可用、运动桥查询异常/超过2秒未返回时，
  保留下一点，每500毫秒重新检查。依赖恢复、定位有效且实际运动开关仍启用时继续。
  等待不会因任意15秒期限转为永久暂停，也不会自行打开运动开关。
- 每次请求有独立标识。迟到的查询、旧action结果和重复回调不能推进或清空新任务。
  可选的末端位置诊断失败也不能撤销已经提交的成功。
- 真正导航失败、非法目标或障碍/未知目标格暂停并保留当前点及后续点。
  修正原因后显式`resume`，从当前点重试。不会绕过禁行格。
- 定位丢失仍沿用原有停车、取消、恢复证明和代价地图刷新流程。
  手动停止或取消清空队列、撤销重试；旧回调无法恢复运动。

点云红色来自显示高度，不参与占用判断。地图范围内的红色位置可以收点、显示编号；
未知或障碍格也会保留原坐标，并明确提示执行受阻。若可走地面在地图中被误标成障碍，
这次传输层修复不会把它强行变成自由空间。

## 官方实现依据

查阅的Nav2 Humble版本为`3c3db59d6969d8ecee8e68468693d006397f4a0c`：

- [WaypointFollower](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_waypoint_follower/src/waypoint_follower.cpp#L171)
  用持续循环推进目标序号，异步发送下一点，并匹配目标标识过滤旧结果。
- [Simple Commander](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_simple_commander/nav2_simple_commander/robot_navigator.py#L163)
  提供FollowWaypoints接口。本仓库保留现有NavigateToPose接口，避免增加服务器与修改核心导航配置。

## 验证与现场区分

`navigation_handoff_smoke.py`通过真实ROS action/service接口和模拟导航端点验证：
一次启用四点连续成功；点间TF/代价地图中断恢复；超过15秒的服务不可用恢复；
查询超时、迟到响应与重复结果；取消等待中的队列；定位暂停后有证明的恢复；
真实ABORTED后显式恢复原点。测试不驱动真实机器狗，也不模拟行走物理。

`navigation_collection_smoke.py`验证两种地图范围、原点和分辨率，Pose/Point两种入口，
未知/净空/占用三种格值：保留三点顺序和精确坐标，执行受阻时零派发，
格子变为自由后仍需显式恢复，再完成三个目标。既有queue/gateway/pose-route/
automatic-resume/localization-pause测试覆盖人工停止、取消、拒绝和恢复边界。

官方SimpleGoalChecker源码测试独立验证30厘米内任意朝向通过、半径外拒绝和新目标重置。
上述检查不能代替现场实机验收，也不能仅凭“到第一点停住”确定实际触发的分支。

本次云端验证结果：104项单元测试、上述7组ROS链路测试及官方到点插件测试全部通过。
同一个handoff测试替换为旧版`f0247d6`操作节点后，能复现第一点成功、剩余3点，
TF恢复后第二点仍不派发的超时失败；新版完成全部4点。

## 机器人更新与核验

在机器人定位有效、站稳时，在第二个SSH终端执行：

```bash
cd /home/unitree/go2_nav
git pull --ff-only
git log -1 --oneline
bash go2_3d/tools.sh restart-navigation dense
```

该重启入口先停止运动、保留当前有效定位，再加载代码；仅git pull不会更新正在运行的Python进程。
若当前定位无效，此入口会拒绝重启，需要先恢复定位。标准模式将dense换成standard。

选择至少三个自由区域点，确认`/navigation/waypoints`显示全部编号、`queued=3`，只启用一次。
应依次出现`SUCCEEDED ... waypoint #1; next waypoint #2`、第二点派发、第三点派发，
最终`completed=3 queued=0`。真实失败应显示暂停，完成数不增加。

```bash
sudo docker logs go2-3d 2>&1 | grep -E 'Waypoint #|Waypoint dispatch|Map goal sent|Nav2 accepted|Nav2 goal completed|Queued goal|Route paused' | tail -60
```
