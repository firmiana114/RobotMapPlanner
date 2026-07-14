# RobotMapPlanner 交接报告

更新时间：2026-07-14（Asia/Singapore）

## 项目整体描述

`RobotMapPlanner` 是面向离线机器人应用的 PCD 地图处理与二维全局路径规划子系统。它读取 PCD v0.7 XYZ 点云，估计地面并按相对高度投影障碍，以凸包划定可规划区域，通过 Web Draft 编辑三态占据覆盖层，发布不可变地图版本，生成 Nav2 风格膨胀代价地图，最终输出固定间距的 XY 点位。

核心技术栈为 C++17、pybind11、FastAPI、SQLite 和原生 HTML/CSS/JavaScript Canvas。运行时不依赖 ROS、PCL、Open3D、GPU、Node 或公网，目标架构为 amd64 和 arm64，最终部署目标是 NVIDIA Orin。

## 核心功能与模块

- `cpp/`：轻量 PCD ASCII/binary 解析、地面估计、凸包、占据投影、覆盖融合、代价地图、地图验证、无穿角 A*、视线简化和等距重采样。
- `src/robot_map_planner/storage.py`：SQLite 元数据、`RMP1` little-endian 栅格文件、原子写入、Draft revision、撤销/重做、发布、激活和规划入口。
- `src/robot_map_planner/api.py`：FastAPI HTTP API、统一错误码、健康检查和静态网页。
- `src/robot_map_planner/static/`：深蓝离线 Web 编辑器，支持画刷、矩形、边界控制点、图层切换、版本、验证、发布和规划显示。
- `src/robot_map_planner/cli.py`：`import|validate|plan|serve` 统一命令，与 HTTP 共用同一 C++ 核心和存储层。
- `tests/`、`cpp/tests/`：API/存储与 C++ 算法测试。
- `Dockerfile`、`compose.yaml`、`scripts/`：CPU-only 双架构构建、运行和冒烟检查。
- `docs/FRONTEND_USER_GUIDE.md`：前端导入、编辑、规划、版本管理与故障处理操作指南。

## 运行入口、配置与数据流

- Web/API：默认 `http://0.0.0.0:28200`；CLI：`robot-map-planner`。
- 数据目录：`RMP_DATA_DIR`，默认 `./data`，容器默认 `/data`。
- 允许导入目录：`RMP_IMPORT_ROOTS`，多个路径以 `:` 分隔。
- 数据流：PCD → 基础占据图 → Draft 覆盖层 → 最终占据图 → 代价地图 → A* → 简化及等距 XY 点位。
- 元数据位于 `catalog.sqlite3`；不可变版本栅格采用带魔数、版本、原点、分辨率和尺寸的 row-major `uint8` 文件。
- 关键依赖和构建配置位于 `pyproject.toml`、`CMakeLists.txt`、`Dockerfile`。

## 当前状态与已验证事实

- 独立首版的核心、服务、Web、CLI、测试和容器交付已实现。
- 已补充与 0.1.0 实际界面一致的中文前端操作文档，包括当前 WSL 部署路径和 Draft 使用限制。
- CTest：1/1 通过；pytest：6/6 通过。既有浏览器验证已覆盖 Draft 编辑、撤销、验证和图层刷新；本轮浏览器控制运行时因 `process` 属性冲突无法初始化，两阶段选择页面的自动化可视化验收尚未确认，但真实 HTTP、DOM、JavaScript 语法和 API 端到端检查均通过。
- Web 导入参数已增加必填、数值范围和“膨胀半径不得小于硬净空”校验；API、存储层和 C++ 核心会返回包含字段和值的明确错误，不再仅返回 `invalid costmap parameters`。
- 分辨率使用 `min=0.01, step=0.01`，代价衰减使用 `min=0.5, step=0.5`；两者的默认值均与 HTML 原生步进基准对齐，不会再触发“请输入有效值”提示。
- 地图导入页采用“先选择 PCD 或已导入地图，再编辑参数”的两阶段流程；未选择来源时参数区禁用。已导入地图会回填原始参数，并通过 `/api/v1/maps/{map_id}/recompile` 从保存的源 PCD 生成新地图，保留原地图、版本和 Draft。
- 目标 PCD `/home/u12297/projects/global_map_20260708_124133.pcd` 的 SHA-256 为 `5c9919abac2ba74376720dbf0e5ff659fc9ce2d30846b4f7981715329f663502`，正确读取 243,037 点。
- 默认参数生成 `751 x 942` 栅格；地面高度约 `-1.31602335 m`；基础占据统计为障碍 21,411、自由 418,055、未知 267,976。
- 选定有效起终点的 amd64 规划输出 137 点、路径长约 `67.77794 m`、扩展 60,804 节点，主机规划约 18 ms。
- amd64 镜像原生通过健康检查；arm64 镜像以 Buildx/QEMU 构建并在 `aarch64` 容器完成导入和规划，约 65 ms。
- amd64 与 arm64 的栅格尺寸、占据统计、点数、路径长度和累计代价一致；点坐标最大差值约 `7.1e-15 m`。
- 普通栅格编辑执行局部代价地图重编译；边界修改执行完整重编译；测试验证局部结果与完整结果一致。

## 日志与一致性

- INFO 日志覆盖服务架构/版本/数据目录、导入 ID/哈希/参数/点数/范围/过滤数/尺寸/耗时、重新编译的源地图 ID/新地图 ID/参数、Draft revision/变更单元、验证结果、发布编译模式/耗时、版本激活和规划结果/耗时；非法导入参数以 WARNING 记录字段上下文，核心导入异常以 ERROR 保留异常链。
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
- 当前没有 Orin SSH 地址、用户和认证配置；已完成 QEMU arm64 验证，原生 Orin 的性能、内存和端到端验收仍待执行。
- 父项目应使用上述 GitHub 地址更新 submodule；获得 Orin 访问后运行导入、编辑、发布、规划与资源指标验收。

## 注意事项

- 不要提交 PCD、运行数据库、日志、栅格二进制、虚拟环境或构建目录。
- 发布必须经过验证，并保持“文件原子落盘 → SQLite 事务切换”；不要改写已发布版本。
- `binary_compressed` PCD 明确不支持，不能静默降级。
