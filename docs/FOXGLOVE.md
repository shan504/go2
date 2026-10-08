# Foxglove 操作流程

使用当前 Foxglove 数据源连接扩展电脑的 8765 端口。
截图中的旧相机话题不存在；删除或修改旧相机面板，不沿用 `/robot0/camera/image_raw`。

## 显示

| 面板 | 设置 |
|---|---|
| 3D | 固定参考系 `map`，显示参考系 `map`，先关闭跟随机器人 |
| 网格上轴 | `Z 向上`，与 ROS 的 XYZ 坐标约定一致 |
| 实时点云 | 开启 `/point_cloud2`，点大小 2–3 px |
| GICP 配准点云 | 开启 `/registered_cloud`，坐标已在 `map`，用于观察已通过配准的三维扫描 |
| 三维地图 | 开启 `/map_cloud`，与实时点云使用不同颜色 |
| 图像 | `/camera/image/compressed` |
| 原始消息 | `/localization/status`、`/localization/valid`、`/operator/status` |
| 导航路径 | `/plan`、`/local_plan`，仅导航模式 |
| 二维栅格 | `/map`，按需开启透明度，避免遮住三维点云 |
| 障碍代价地图 | `/local_costmap/costmap`、`/global_costmap/costmap`，按需开启 |

观察稳定的三维场景时，先只开启 `/registered_cloud` 和 `/map_cloud`。
这两个话题保留 XYZ，颜色字段选 `z`（高度）；它们没有 `intensity` 字段。
`/registered_cloud` 是实际 GICP 配准结果，只在配准通过时更新；`/map_cloud` 是保存到 PCD 的地图。
地图点云衰减时间保持 0，避免把每次重复发布的全图叠加。
实时 `/point_cloud2` 是机身坐标系单帧扫描，衰减时间 0 时显示每帧采样变化；
在稳定 `odom`/`map` 参考系下可以设置 0.5–1 秒观察扫描覆盖，但显示累积不等同于建图或定位成功。

地图存储分辨率现为 6 厘米，配准仍使用 15 厘米降采样以控制 ARM CPU 消耗。
即使机器人静止，也会每隔 2 秒加入通过匹配检查的扫描以补充地图覆盖；数据预算仍为 20 万地图点。
传感器/配准不稳定时先运行只读诊断，并让机器狗静止。
诊断中的 `z_span` 检查实际高度范围，TF variation 分别显示里程计与 GICP 校正的位姿变化；
另外列出 `/tf` 发布节点供排查重复节点；这份列表不表示各节点发布了哪一条 TF 边。
诊断也读取原生 `cloud_deskewed`、`voxel_map`
和 `uslam` 点云/里程计；话题存在不等于有数据，也不能直接假定其坐标系与本地图一致。
截图无法确定抖动原因，不应靠延长衰减时间掩盖位姿跳动。

定位刚启动但尚未设置初始位姿时，`map → odom` 不存在；可暂时选 `odom` 查看实时点云，
地图仍在 `map` 下。设置位姿并匹配通过后切回 `map`。不要添加固定的 `map → odom` 来掩盖问题。

相机使用官方 GetImageSample，最高约 2 Hz。只有日志出现
`First camera sample published` 且图像实际刷新才算接通。RPC 超时或非零状态码需要检查固件接口，
不能靠改话题名解决。

点云或三维地图未显示时，在第二个 SSH 终端运行：

```bash
cd ~/go2_nav
git pull --ff-only
bash go2_3d/tools.sh diagnose
```

这会在当前容器中只读采样 12 秒，输出原始与桥接消息数、frame、时间戳、有效 XYZ、
地图数据及两段 TF。文件通过目录挂载生效，不需要重建镜像或重启建图。
Foxglove 订阅日志仅证明订阅建立，不证明收到数据。可暂时将固定/显示参考系都设成
`base_link`，展开“主题”并开启 `/point_cloud2` 检查数据接收；建图成功后将固定/显示参考系都切回
`map` 并开启 `/registered_cloud`、`/map_cloud`，让显示保持在世界坐标系中。

## 建图与保存

SSH 启动 `bash go2_3d/run.sh mapping` 后，静置等待 `/map_cloud`，再用遥控器缓慢走动。
查看地图是否连续且墙体对齐；本模式不发送运动命令。

在 Foxglove 添加 **发布（Publish）面板**：

| 用途 | 话题 | 消息类型 | JSON 内容 |
|---|---|---|---|
| 保存地图 | `/mapping/save` | `std_msgs/msg/Empty` | `{}` |
| 启用运动 | `/control/enable` | `std_msgs/msg/Bool` | `{"data":true}` |
| 停止运动 | `/control/enable` | `std_msgs/msg/Bool` | `{"data":false}` |
| 取消导航 | `/navigation/cancel` | `std_msgs/msg/Empty` | `{}` |

不同 Foxglove 版本可能把类型显示为 `std_msgs/Empty` 等，选择服务器实际公布的类型即可。
设置为手动单次发布，避免自动重复保存或重复启用。

保存时点击发布，检查 `/operator/status` 中出现 `Save success=True`，并包含保存目录。
成功意味着同一个目录同时包含 `map.pcd`、`nav.pgm`、`nav.yaml` 和 `metadata.yaml`。
失败时保留诊断，不把部分生成文件用作导航地图。

## 初始化定位/重定位

保存后通过 SSH 结束建图并运行 `bash go2_3d/run.sh navigation`。
此时加载 `maps/latest` 的三维 PCD 与对应栅格，运动默认关闭。

在 3D 面板工具设置中：

- 初始位姿工具使用 `/initialpose`，类型 `geometry_msgs/msg/PoseWithCovarianceStamped`。
- 固定参考系使用 `map`，点击地图上的实际位置并拖动朝向。
- 该地图的原点是建图第一帧机身位置；平地机身 `z` 大致为 0。
- 如果当前 Foxglove 版本不支持该工具，使用 Publish 面板发布该类型，填写真实位置与朝向。

初始位姿示例（只有回到建图起点、同一朝向时才适用）：

```json
{
  "header":{"stamp":{"sec":0,"nanosec":0},"frame_id":"map"},
  "pose":{
    "pose":{"position":{"x":0,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}},
    "covariance":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  }
}
```

等待 `/localization/valid` 连续为 `true`，查看 `/localization/status` 的 fitness/RMSE，
并检查当前点云与三维地图是否重合。单独一个 `true` 不保证场景无歧义。
重新初始化同样使用 `/initialpose`；目标取消前不要在运动中重新设定位姿。

## 导航

确认定位与现场状态后，在启用运动面板单次发布 `{"data":true}`。
查看 `/operator/status` 确认参数设置成功。随后使用 3D 面板的发布位姿工具：

- 目标话题 `/goal_pose`，类型 `geometry_msgs/msg/PoseStamped`，参考系 `map`。
- 点击已观察到的空地并拖动目标朝向。
- 首次选近距离目标，观察 `/plan`、局部障碍和机器人响应。

新增的 `operator_bridge.py` 把 `/goal_pose` 转换为 `/navigate_to_pose` action。
`/operator/status` 应显示 Nav2 接受目标；定位无效、非导航模式或目标参考系错误时会拒绝。
有进行中的目标时先取消，等待 action 完成，再选新目标。

取消导航发布 `/navigation/cancel`；停止运动发布 `/control/enable` 为 `false`。
定位匹配失败或失去传感器数据时，运动桥会发 StopMove，同时请求取消目标。
软件停止接口需要先在现场验证，遥控器保留在操作人员手中。

首次运行和 mapping/localization/navigation 模式切换通过 SSH 完成；
当前没有把远程重启容器接入 Foxglove。你可将上述面板保存到现有 Foxglove 布局供以后使用。
