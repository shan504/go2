# Foxglove 操作流程

使用当前 Foxglove 数据源连接扩展电脑的 8765 端口。
截图中的旧相机话题不存在；删除或修改旧相机面板，不沿用 `/robot0/camera/image_raw`。

## 显示

| 面板 | 设置 |
|---|---|
| 3D | 固定参考系 `map`，显示参考系 `map`，先关闭跟随机器人 |
| 实时点云 | 开启 `/point_cloud2`，点大小 2–3 px |
| 三维地图 | 开启 `/map_cloud`，与实时点云使用不同颜色 |
| 图像 | `/camera/image/compressed` |
| 原始消息 | `/localization/status`、`/localization/valid`、`/operator/status` |
| 导航路径 | `/plan`、`/local_plan`，仅导航模式 |
| 二维栅格 | `/map`，按需开启透明度，避免遮住三维点云 |
| 障碍代价地图 | `/local_costmap/costmap`、`/global_costmap/costmap`，按需开启 |

定位刚启动但尚未设置初始位姿时，`map → odom` 不存在；可暂时选 `odom` 查看实时点云，
地图仍在 `map` 下。设置位姿并匹配通过后切回 `map`。不要添加固定的 `map → odom` 来掩盖问题。

相机使用官方 GetImageSample，最高约 2 Hz。只有日志出现
`First camera sample published` 且图像实际刷新才算接通。RPC 超时或非零状态码需要检查固件接口，
不能靠改话题名解决。

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
