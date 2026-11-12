# 本地点位路线

`route_cli.py` 将完整点位列表写入现有 `/maps` 挂载中的 `routes/NAME.json`，
宿主机对应 `~/go2_nav/maps/routes/NAME.json`。文件包含版本、地图身份、按顺序排列的
位置和四元数；不包含执行中的控制指令或自动运动授权。

| 命令 | 行为 |
|---|---|
| `bash go2_3d/tools.sh route-save NAME` | 保存本轮全部点位；默认拒绝覆盖同名文件 |
| `bash go2_3d/tools.sh route-save NAME --overwrite` | 明确更新已保存路线 |
| `bash go2_3d/tools.sh route-list` | 本地列出路线、点数和地图标签 |
| `bash go2_3d/tools.sh route-show NAME` | 本地显示完整文件 |
| `bash go2_3d/tools.sh route-load NAME` | 整体加载到空队列，不启用运动 |
| `bash go2_3d/tools.sh route-run NAME` | 加载并明确请求启用原导航队列 |

首次安装后要重新启动导航节点，再选择点位保存。以后先启动正确地图的导航、等待定位，
再执行 `route-run` 即可；不需要连接互联网或打开 Foxglove。已有镜像、地图、ROS/DDS
链路必须可用。使用默认地图起点初始化时，仍须把狗放在建图起点并保持原朝向。

## 数据与执行边界

- `Operator.planned_route` 保存本轮完整计划；执行队列可以推进或清空，完整计划仅供显式保存。
  重启后执行队列为空，保存文件仍在；读文件不会自动恢复历史运动权限。
- 文件内容绑定 `/runtime/navigation_map` 的点云、占据图及坐标相关元数据。
  身份在节点启动时计算一次，不在控制循环里反复读取大地图。排除路径、时间戳等，
  原样复制可复用；几何、栅格或坐标变换变化会拒绝加载。
- 导入整条路线前验证地图、格式、有限数值、四元数和每个点的地图边界；失败不改现有队列。
  静态障碍/未知格可以保存为计划，真正派发时仍按原逻辑检查实时地图、TF、定位和运动开关。
- 新指令走 `/navigation/route/command`，回执走 `/navigation/route/result`，均为可靠、
  非持久 ROS String 消息。UUID 关联请求，节点缓存最近128个请求，重发同一请求不会重复执行。
  同一 UUID 携带不同内容会被拒绝。
- CLI 每次只发送一次请求。超时不能推断请求未执行，因此提示先查看 `/operator/status`，
  不自动重试运动命令。`START REQUESTED` 只表示请求已接收，不等于底盘已运动或路线已完成。
- 手动停止/取消仍清空执行队列；文件保留。真实 Nav2 失败仍暂停在该点，`resume` 重试该点；
  保存、加载和文件存在本身都不授权自动恢复。
- 文件原子写入并同步到磁盘，默认不覆盖；名称和路径受限，损坏、超限及非有限坐标会拒绝。

## 验证范围

`tests/test_route_store.py` 检查持久化、重复读取、两个地图、原始/调平坐标身份、
复制归档、并发保存及异常文件；`tests/test_route_cli.py` 检查保存/加载/运行意图区分、
本地查看、非法输入和超时不重试。

`tests/offline_routes_smoke.py` 使用实际 ROS2 节点、命令话题、参数服务和 Nav2 action 协议，
由测试服务器返回目标结果，验证三个目标顺序执行、失败暂停及重试、人工停止、重复请求、
错误地图整体拒绝，以及销毁重建操作节点后从同一文件执行三点。CLI 子进程参与端到端测试。
它不运行真实规划器、底盘或传感器，因此不能代替农场实机验收。
同一端到端测试也在 Docker `--network none` 的 ROS2 Humble 容器中通过，
验证文件读取、命令回执及三点队列执行不依赖互联网；ROS通信仅发生在容器本地。

纯 Python 测试：

```bash
python3 -m unittest discover -s tests -p 'test_route_*.py'
```

在已安装项目 ROS 依赖的隔离容器中：

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
python3 tests/offline_routes_smoke.py
```
