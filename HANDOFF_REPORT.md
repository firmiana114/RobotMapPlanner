# RobotMapPlanner 交接报告

更新时间：2026-07-14（Asia/Singapore）

## 项目整体描述

`RobotMapPlanner` 是面向离线机器人应用的 PCD 地图处理与二维全局路径规划子系统。它读取 PCD v0.7 XYZ 点云，估计地面并按相对高度投影障碍，以凸包划定可规划区域，通过 Web Draft 编辑三态占据覆盖层，发布不可变地图版本，生成 Nav2 风格膨胀代价地图，最终输出固定间距、带二维朝向四元数和导航 mode 的完整位姿点位。

核心技术栈为 C++17、pybind11、FastAPI、SQLite 和原生 HTML/CSS/JavaScript Canvas。运行时不依赖 ROS、PCL、Open3D、GPU、Node 或公网，目标架构为 amd64 和 arm64，最终部署目标是 NVIDIA Orin。

## 核心功能与模块

- `cpp/`：轻量 PCD ASCII/binary 解析、地面估计、凸包、占据投影、覆盖融合、代价地图、地图验证、无穿角 A*、视线简化和等距重采样。
- `src/robot_map_planner/storage.py`：SQLite 元数据、`RMP1` little-endian 栅格文件、原子写入、Draft revision、撤销/重做、发布、激活和规划入口。
- `src/robot_map_planner/api.py`：FastAPI HTTP API、统一错误码、健康检查和静态网页。
- `src/robot_map_planner/navigation.py`：RobotAbrainOffline NavBridge 健康检查、HTTP 当前位姿读取、起点安全校验和逐点路径执行状态机。
- `src/robot_map_planner/static/`：深蓝离线 Web 编辑器，支持画刷、矩形、边界控制点、图层切换、版本、验证、发布和规划显示。
- `src/robot_map_planner/cli.py`：`import|validate|plan|serve` 统一命令，与 HTTP 共用同一 C++ 核心和存储层。
- `tests/`、`cpp/tests/`：API/存储与 C++ 算法测试。
- `Dockerfile`、`compose.yaml`、`scripts/`：CPU-only 双架构构建、运行和冒烟检查。
- `deploy/robot-map-planner.service`：AGX Orin `/mnt/ssd/gt/RobotMapPlanner` 原生部署的用户级 systemd 单元。
- `docs/FRONTEND_USER_GUIDE.md`：前端导入、编辑、规划、版本管理与故障处理操作指南。

## 运行入口、配置与数据流

- Web/API：默认 `http://0.0.0.0:28200`；CLI：`robot-map-planner`。
- 数据目录：`RMP_DATA_DIR`，默认 `./data`，容器默认 `/data`。
- 允许导入目录：`RMP_IMPORT_ROOTS`，多个路径以 `:` 分隔。
- 数据流：PCD → 基础占据图 → Draft 覆盖层 → 最终占据图 → 代价地图 → A* → 安全采样 → 折叠共线点 → 二维朝向四元数转向位姿点位。
- 元数据位于 `catalog.sqlite3`；不可变版本栅格采用带魔数、版本、原点、分辨率和尺寸的 row-major `uint8` 文件。
- 关键依赖和构建配置位于 `pyproject.toml`、`CMakeLists.txt`、`Dockerfile`。
- AGX Orin 运行入口为用户级 `robot-map-planner.service`，数据目录为项目下 `data/`，允许导入目录为项目下 `imports/`，服务地址为 `http://192.168.1.21:28200`。
- NavBridge 默认地址为 `http://127.0.0.1:28180`，可用 `RMP_NAV_BRIDGE_URL` 及 `RMP_NAV_*` 超时、轮询、起点容差环境变量调整。

## 当前状态与已验证事实

- 独立首版的核心、服务、Web、CLI、测试和容器交付已实现。
- 已补充与 0.1.0 实际界面一致的中文前端操作文档，包括当前 WSL 部署路径和 Draft 使用限制。
- 本机与 Orin 的 CTest 均为 1/1、pytest 均为 17/17 通过。Orin `GET /api/v1/navigation/pose` 已通过 NavBridge HTTP 返回 `localized=true` 的实时七元组；浏览器自动化仍因 `process` 属性冲突无法初始化，JavaScript 可视交互验收未完成。
- Web 导入参数已增加必填、数值范围和“膨胀半径不得小于硬净空”校验；API、存储层和 C++ 核心会返回包含字段和值的明确错误，不再仅返回 `invalid costmap parameters`。
- 分辨率使用 `min=0.01, step=0.01`，代价衰减使用 `min=0.5, step=0.5`；两者的默认值均与 HTML 原生步进基准对齐，不会再触发“请输入有效值”提示。
- 地图导入页采用“先选择 PCD 或已导入地图，再编辑参数”的两阶段流程；未选择来源时参数区禁用。已导入地图会回填原始参数、默认追加“ `_参数版本` ”名称，并通过 `/api/v1/maps/{map_id}/recompile` 从保存的原始 PCD 创建独立地图；原地图、版本、Draft 和人工编辑保持不变。列表显示本地化创建时间，并提供带确认弹窗的删除按钮。
- 编辑画布保留十字光标，并按画刷栅格半径和画布缩放比例实时显示跟随指针的圆形轮廓；矩形工具、边界模式或离开画布时自动隐藏。
- 规划页可分别设置起点和终点朝向（角度），画布以箭头实时显示方向。规划 API/CLI 使用弧度；首尾点使用指定 yaw，中间点使用当前点指向下一点的路径切线 yaw。
- HTTP、CLI 和网页导出的每个路径点均包含 `x/y/z/ox/oy/oz/ow/mode`；二维约束固定 `z=ox=oy=0`，`oz=sin(yaw/2)`、`ow=cos(yaw/2)`，`mode` 默认 `1`。网页可下载仅包含完整点位序列的 JSON 数组。
- 规划页已接入 RobotAbrainOffline NavBridge：定位成功时通过 NavBridge HTTP `/current_pose` 读取容器内 ROS2 位姿，避免依赖 Orin 宿主 ROS2 CLI/DDS；离线、未定位或位姿过期自动回退地图点击。规划路径经确认和 `0.75 m` 起点距离校验后，从第二点开始逐点调用 `/go_to_async` 并轮询 `/go_to_status`，前端显示执行进度和失败码。
- 最终输出会删除重复点和同向共线采样点，只保留起点、方向变化点和终点；前端为每个点显示序号和朝向箭头，起终点使用更大的高对比圆环、箭头和文字。底层等距采样仍用于线段安全验证，不直接作为机器人点位导出。
- 目标 PCD `/home/u12297/projects/global_map_20260708_124133.pcd` 的 SHA-256 为 `5c9919abac2ba74376720dbf0e5ff659fc9ce2d30846b4f7981715329f663502`，正确读取 243,037 点。
- 默认参数生成 `751 x 942` 栅格；地面高度约 `-1.31602335 m`；基础占据统计为障碍 21,411、自由 418,055、未知 267,976。
- 选定有效起终点的 amd64 规划输出 137 点、路径长约 `67.77794 m`、扩展 60,804 节点，主机规划约 18 ms。
- amd64 镜像原生通过健康检查；arm64 镜像以 Buildx/QEMU 构建并在 `aarch64` 容器完成导入和规划，约 65 ms。
- amd64 与 arm64 的栅格尺寸、占据统计、点数、路径长度和累计代价一致；点坐标最大差值约 `7.1e-15 m`。
- 普通栅格编辑执行局部代价地图重编译；边界修改执行完整重编译；测试验证局部结果与完整结果一致。
- 已通过 `ssh agx-orin` 部署到 `/mnt/ssd/gt/RobotMapPlanner`：原生 `aarch64` 扩展构建成功，用户级 systemd 服务 enabled/active，`Linger=yes`，Windows 访问首页、静态资源及健康检查均为 HTTP 200。真实规划 API 已验证 90°/-90° 首尾朝向，直线路径从 3 个安全采样点折叠为 2 个完整位姿转向点且 `mode=1`；服务日志包含 yaw/mode、`sampled_points=3` 和 `turning_points=2`。

## 日志与一致性

- INFO 日志覆盖服务架构/版本/数据目录、导入、Draft、发布、版本、规划、导航请求排队、逐点到达和路径完成；非法参数及 ROS2 位姿读取失败以 WARNING 记录必要上下文，核心、文件和导航执行异常以 ERROR 保留异常链。
- 异常日志保留地图、版本或 Draft 上下文并使用异常链，不记录完整点云、完整栅格或密钥。
- A* 的启发式和步进使用固定点整数，并使用稳定 tie-break，避免 amd64/arm64 路径分叉。
- 规划默认最大可通行代价为 `0`，完整避开膨胀区；视线简化使用 supercover 栅格检查，并在重采样输出中保留安全拐点，禁止线段擦角或削角穿越高代价/阻塞栅格。
- 规划 INFO 日志包含 `max_traversable_cost`，便于诊断路径是否允许进入软代价区。

## 外部依赖与常用命令

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e '.[test]'
cmake -S . -B build-ctest -DRMP_BUILD_PYTHON=OFF -DRMP_BUILD_TESTS=ON
cmake --build build-ctest && ctest --test-dir build-ctest --output-on-failure
pytest
robot-map-planner serve --host 0.0.0.0 --port 28200
RMP_PLATFORMS=linux/amd64,linux/arm64 bash scripts/build_multiarch.sh
```

## 发布状态、阻塞问题与下一步

- GitHub 公开仓库已创建：`https://github.com/firmiana114/RobotMapPlanner`；本地 `main` 以该仓库为 `origin`。
- AGX Orin SSH 别名为 `agx-orin`，主机地址为 `192.168.1.21`；源码、虚拟环境、数据和导入目录均位于 `/mnt/ssd/gt/RobotMapPlanner`。
- AGX Orin 是当前唯一运行服务器；本机 WSL 仅用于编辑、测试与 Git 提交，不应再启动 28200 服务。每次提交后必须同步 Orin 并重启 `robot-map-planner.service`。
- Orin 无法直连 Docker Registry；本轮曾以 SSH 反向转发接入本机 7897 代理，但为避免修改共享 Docker 守护进程，最终采用原生 Python/systemd 部署。后续如需 Docker 构建，应继续使用反向代理或由管理员配置守护进程代理。
- 原生 Orin 的页面与测试已验收；完整目标 PCD 的 Orin 导入、编辑、发布、规划性能和资源指标仍待执行。
- Orin NavBridge `28180` 已运行；当前位姿改由其 HTTP 接口提供。机器人运动仍未执行联调，不得在无人监护时测试运动。

## 注意事项

- 不要提交 PCD、运行数据库、日志、栅格二进制、虚拟环境或构建目录。
- 发布必须经过验证，并保持“文件原子落盘 → SQLite 事务切换”；普通编辑不得改写已发布版本，参数重新编译必须创建独立地图，不得修改或删除源地图的版本、Draft 和人工编辑。
- `binary_compressed` PCD 明确不支持，不能静默降级。
