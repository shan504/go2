# Go2 EDU 三维点云与前置相机预览补丁

复用已安装的 go2-ros2-sdk:edu 镜像及 ~/go2_edu_fix，不再下载安装依赖。
本补丁是传感器预览和录像入口，不包含三维全局建图、PCD 导出或 GICP 定位节点。
已做本地响应解析与脚本检查；实际相机 RPC 是否响应，需在你的固件上运行确认。

Windows 下载 ZIP 至 Downloads，然后 PowerShell 上传：

```powershell
scp "$env:USERPROFILE\Downloads\go2_edu_sensor_patch.zip" unitree@192.168.123.18:/home/unitree/
```

机器人 SSH 终端：

```bash
cd ~
python3 -m zipfile -e go2_edu_sensor_patch.zip .
bash ~/go2_edu_sensor_patch/run.sh
```

脚本会停止名为 go2-sdk 的原开发容器，开启新的传感器预览。
机器狗原系统服务、SDK 镜像和已保存地图不会修改或删除。
本入口不启动 slam_toolbox、AMCL 或 Nav2，也不发送运动指令。

Foxglove 重连 ws://192.168.123.18:8765，按以下设置：

- 3D 面板固定坐标系 odom；启用 /point_cloud2，点大小先设 2–3 px。
- 暂时关闭旧的 /map 与代价地图图层，避免二维平面遮挡三维点云。
- 图像面板选择 /camera/image/compressed。原 /robot0/camera/image_raw 不适用于此入口。
- 点云是每一帧的 XYZ 数据；Foxglove 延长显示时长产生的累积效果不等于已保存的全局 PCD 地图。

另一个机器人 SSH 终端查看传感器状态：

```bash
bash ~/go2_edu_sensor_patch/tools.sh check
```

相机日志出现 First camera sample published 才表示成功取图。
如果出现 videohub sample timeout 或非零 status，请提供日志，不要假定 JPEG 流已接通。
当前取图走 Unitree 官方 GetImageSample API 1001，最高 2 帧/秒，属于图像采样预览；
不是完整解码 /frontvideostream 的高帧率视频。
它只提供二维图像面板，未提供相机内参及外参，不能直接做三维图像投影或视觉定位。

需要先留存三维数据时：

```bash
bash ~/go2_edu_sensor_patch/tools.sh record
```

用手机缓慢走动；录制终端 Ctrl+C 正常结束。数据存入 ~/go2_edu_fix/maps/3d_sensors_时间戳/。
这是 rosbag 传感器记录，不是完成的三维 SLAM 地图。

后续目标链路：一致坐标系的三维 PCD 地图 + 当前点云 + 连续里程计 -> GICP map->odom -> Nav2。
GICP 精配准通常需要大致初始位姿；无初始位置的全局重定位需要另外的全局匹配步骤。
Nav2 的二维障碍栅格可由同一三维地图生成，必须与 PCD 共用 map 坐标原点和朝向。
GICP 成为 map->odom 发布者后，不能再让 AMCL/SLAM 发布同一段 TF。
必须先确认有可用的三维地图构建链路和匹配质量，再允许导航运动。

官方相机接口来源：
https://github.com/unitreerobotics/unitree_sdk2_python/tree/master/unitree_sdk2py/go2/video
https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_video_client.cpp
