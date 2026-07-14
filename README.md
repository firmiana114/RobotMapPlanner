# RobotMapPlanner

RobotMapPlanner 是一个离线 PCD 地图处理、Web 占据地图编辑、Nav2 风格代价地图生成和二维全局路径规划服务。核心算法使用 C++17/pybind11，管理接口使用 FastAPI，网页使用原生 Canvas；运行时不依赖 ROS、PCL、Open3D、GPU 或公网资源。

## 数据链路

```text
PCD → 基础占据地图 → Draft 覆盖编辑 → 发布版本 → 代价地图 → A* → 等距 XY 点位
```

PCD 空白区域的语义为：自动凸包边界内无障碍点的栅格视为自由，边界外为未知。基础地图不可修改，人工修改以 `INHERIT/FORCE_FREE/FORCE_OCCUPIED` 覆盖层保存。

## 本地安装与运行

```bash
cd /home/u12297/projects/RobotMapPlanner
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[test]'
pytest

RMP_DATA_DIR=./data \
RMP_IMPORT_ROOTS=/home/u12297/projects \
robot-map-planner serve --host 0.0.0.0 --port 28200
```

浏览器访问 `http://localhost:28200`。

## CLI

```bash
robot-map-planner --data-dir ./data import /path/to/map.pcd --name global_map
robot-map-planner --data-dir ./data validate draft_xxx
robot-map-planner --data-dir ./data plan ver_xxx --start 0 0 --goal 8 5 --output path.json
```

## Docker 与多架构

```bash
docker build -t robot-map-planner:0.1.0 .
docker compose up -d

# 同时验证 amd64/arm64 构建缓存
RMP_PLATFORMS=linux/amd64,linux/arm64 bash scripts/build_multiarch.sh

# 单平台加载到本地
RMP_PLATFORMS=linux/amd64 bash scripts/build_multiarch.sh
bash scripts/smoke_container.sh
```

镜像为 CPU-only，目标平台是 `linux/amd64` 和 NVIDIA Orin 使用的 `linux/arm64`。

## API

- `POST /api/v1/maps/import`
- `GET /api/v1/maps`
- `POST /api/v1/maps/{map_id}/recompile`
- `POST /api/v1/maps/{map_id}/drafts`
- `PATCH /api/v1/drafts/{draft_id}`
- `POST /api/v1/drafts/{draft_id}/undo|redo|validate|publish`
- `POST /api/v1/versions/{version_id}/activate`
- `GET /api/v1/versions/{version_id}/grid/{layer}`
- `GET /api/v1/drafts/{draft_id}/grid/{layer}`
- `GET /api/v1/versions/{version_id}/tiles/{layer}/{x}/{y}`
- `POST /api/v1/versions/{version_id}/plan`
- `GET /healthz`

API 详细字段可以启动服务后查看 `/docs`。

前端页面的完整操作步骤、参数说明和常见错误处理参见 `docs/FRONTEND_USER_GUIDE.md`。

## 当前边界

- 支持 PCD v0.7 ASCII 和 binary XYZ；明确拒绝 `binary_compressed`。
- 只做静态二维地图和全局路径点位，不包含机器人、定位、SLAM、控制器、动态障碍和完整三维规划。
- 首版为单进程、单用户离线服务，不提供认证或多人协同。
- 规划默认 `max_traversable_cost=0`，禁止进入完整膨胀代价区；可在 API 或规划页面显式提高阈值以启用软代价通行。

## 已验证基线

- `/home/u12297/projects/global_map_20260708_124133.pcd`：243,037 点，生成 `751 x 942` 栅格。
- amd64 原生与 arm64/QEMU 均完成导入、发布和规划；同一输入返回 137 个点，坐标最大差值小于 `1e-12 m`。
- 普通占据编辑只重算变更包围盒及膨胀邻域；边界修改自动回退到完整代价地图编译。
- 原生 Orin 验收尚需目标设备的 SSH 地址和认证信息。
