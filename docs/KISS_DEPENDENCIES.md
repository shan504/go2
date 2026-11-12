# KISS-SLAM 固定版本和隔离环境

本分支使用原作者 [PRBonn/kiss-slam](https://github.com/PRBonn/kiss-slam)
的论文复现标签 **IROS25**，固定提交：

`db81ee31b59bc7a18af3fec8641d5867646ab57a`

该提交的包版本字符串是 `0.0.2`，但它**不等同于 PyPI 的 0.0.2 二进制包**。
源码在发布后修改过每个体素计算法向量所需的点数。本分支从上述提交编译
原作者的 C++ 扩展，不替换成同版本号的任意 wheel，也不修改上游算法源码。

## 固定依赖

| 依赖 | 版本 | 用途 |
| --- | --- | --- |
| KISS-SLAM | IROS25 / 上述 SHA | 子图、闭环验证、图优化接口 |
| MapClosures | 2.0.2 | 激光地点检索 |
| Open3D | 0.19.0 | 上游后端的 tensor ICP；仅在隔离后端使用 |
| kiss-icp | 1.2.3 | KISS-SLAM 的导入依赖；不启动或替换现有 GICP 前端 |
| NumPy | 1.26.4 | MapClosures 2.0.2 要求 NumPy 小于 2 |
| rosbags | 0.11.0 | 离线读取 ROS1/ROS2 录制数据，不运行 ROS 节点 |

`requirements-kiss.in` 记录显式选择，`requirements-kiss.lock` 包含传递依赖的
精确版本及发布包 SHA256。锁文件针对 Python 3.10–3.12 解析；Open3D 不同
Python wheel 的元数据有所差异，因此显式加入了 `ipywidgets` 和 `nbformat`。

KISS-SLAM 固定源码中的 CMake 再获取以下原作者依赖：Eigen 3.4.0、
g2o `20241228_git`、SuiteSparse 7.10.1、Bonxai 0.6.0、robin-map 1.4.0。
这些沿用上游固定标签和补丁；不使用系统中不确定版本的 g2o/Eigen。
KISS-SLAM 本身使用 MIT 许可证，外部依赖分别遵循其许可证，参见上游
[`3rdparty`](https://github.com/PRBonn/kiss-slam/tree/db81ee31b59bc7a18af3fec8641d5867646ab57a/kiss_slam/kiss_slam_pybind/3rdparty)。

## 独立安装

仅在本功能分支的 worktree 执行：

```bash
KISS_PYTHON=python3 bash go2_3d/setup_kiss_env.sh
.venv-kiss/bin/python go2_3d/check_kiss_env.py --smoke
```

脚本只创建本 worktree 的 `.venv-kiss` 和 `.kiss-deps`；不安装系统软件、
不修改旧虚拟环境、不启动 ROS、导航或运动服务。已有环境缺少本脚本的
所有权标记时会拒绝修改。需要 C/C++ 编译器、Git、patch、Python 开发头文件。
可通过 `KISS_VENV` / `KISS_DEPS` 指定**新的独立位置**。

针对 GCC 14，Bonxai 的代码在 `-Werror` 下触发 `maybe-uninitialized`
编译错误。构建脚本仅将这一类诊断降为警告
（`-Wno-error=maybe-uninitialized`），没有更改算法源码。现有地图重建
仍使用项目原来的地图维护接口，不调用 Bonxai 的二维地图生成器。

构建成功后，`kiss-provenance.json` 存在该 venv 根目录，记录源提交、核心
依赖版本及安装后的 Python/C++ 扩展文件 SHA256。`check_kiss_env.py` 和后端
启动检查会拒绝没有来源记录、版本不符或文件已被替换的安装。

## 可选镜像

`Dockerfile.kiss-backend` 以既有 `go2-3d:edu` 为基础，增加独立的
`/opt/kiss-venv`。不改变系统 Python/Open3D；前端 GICP 留在原解释器运行，
KISS 后端使用独立解释器进程。仅构建新镜像不会更新或重启旧容器。

```bash
docker build -f Dockerfile.kiss-backend -t go2-3d:kiss-backend .
```

在有托管代理 CA 的云环境，构建时额外传入
`--secret id=proxy_ca,src=/etc/ssl/certs/ca-certificates.crt`，保持 TLS 校验。
镜像不内置任何地图、机器人地址或传感器标定。

## Jetson ARM64 限制

**当前安装脚本和依赖验证支持 Linux x86_64，不能宣称已完成 Jetson 部署。**
核对 PyPI 后，Open3D 0.19.0、MapClosures 2.0.2、kiss-icp 1.2.3 均没有
CPython 3.10 的 aarch64 wheel；Open3D 0.19.0 也没有 PyPI 源码发行包。
因此普通 `pip install` 不能直接完成同版本的 Jetson 安装。安装脚本会在
修改环境前明确拒绝不支持的架构，不偷偷降低版本。

ARM64 后续需要在单独的 Jetson 构建环境编译相同版本的依赖并记录 wheel
哈希，形成单独的 ARM64 锁文件，再执行同样的来源检查和离线回放测试。
Open3D 官方 0.19.0 源码提交为
`1e7b17438687a0b0c1e5a7187321ac7044afe275`，
[固定版本编译说明](https://github.com/isl-org/Open3D/blob/v0.19.0/docs/compilation.rst)
提供 `pip-package` 构建目标。该步骤未在本次云端 x86_64 环境验证，
不能把官方源码可用等同于 Go2 的 JetPack/ARM64 兼容性已经通过。

## 本次环境验证范围

独立 Python 3.12 环境，以及新建 Ubuntu 22.04 / ROS2 Humble 容器内的
Python 3.10 环境，均已实际从固定源提交编译原生扩展，并通过：

- MapClosures 实例创建和原生库加载；
- 原作者 g2o 两位姿约束优化；
- 原作者体素地图插入/读取；
- 安装文件来源校验以及 `pip check`。

新测试容器中再次检查系统解释器仍使用 Open3D **0.14.1**，隔离解释器使用
**0.19.0**；安装后断开该测试容器的 Docker bridge 网络再进行后续 ROS
集成测试。没有在原有运行容器或原虚拟环境中安装依赖。

这些是依赖/接口检查，不是农场闭环成功或性能提升的证据。
完整算法与 ROS 集成测试结果另见 [主接入文档](KISS_BACKEND.md)。
