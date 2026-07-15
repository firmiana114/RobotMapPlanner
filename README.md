# RobotMapPlanner

RobotMapPlanner 是一个离线 PCD 地图处理、Web 占据地图编辑、Nav2 风格代价地图生成和二维全局路径规划服务。核心算法使用 C++17/pybind11，管理接口使用 FastAPI，网页使用原生 Canvas；运行时不依赖 ROS、PCL、Open3D、GPU 或公网资源。

## 数据链路

```text
PCD → 基础占据地图 → Draft 覆盖编辑 → 发布版本 → 代价地图 → A* → 转向位姿点位
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
robot-map-planner --data-dir ./data plan ver_xxx --start 0 0 --start-yaw 0 \
  --goal 8 5 --goal-yaw 1.5708 --mode 1 --output path.json
```

`--start-yaw` 和 `--goal-yaw` 使用弧度。HTTP API 与 CLI 输出的每个 `points` 元素都包含
`x/y/z/ox/oy/oz/ow/mode`；二维规划固定 `z=ox=oy=0`，`oz/ow` 是 yaw 对应的四元数，
`mode` 默认为 `1`。网页中的朝向输入使用角度，并可直接下载仅包含完整点位序列的 JSON 数组。
最终输出会折叠重复点和同向共线采样点，只保留起点、实际转向点和终点；底层仍保留安全采样用于验证线段不会穿越障碍。

规划请求可通过 `waypoints` 传入最多 50 个有序途径点。服务按“起点 → W1 → W2 → … → 终点”逐段运行 A* 并合并结果；每个途径点即使与前后路径共线也会保留在最终导航点序列中。响应包含请求/吸附后的途径点和每段统计，网页支持添加、撤销、清空并用 `W1/W2/...` 标记顺序。

规划页面可复用同机 `RobotAbrainOffline` 的 NavBridge（默认 `http://127.0.0.1:28180`）：定位成功时“点击设置起点”通过 NavBridge `GET /current_pose` 读取容器内 ROS2 `/current_pose`；规划完成后“按路径行走”会在安全确认和起点距离校验通过后，逐点调用 NavBridge。NavBridge 离线、没有定位消息或位姿过期时不会下发运动。

路径执行期间服务以默认 5 Hz 持续读取同一地图坐标位姿，在规划画布上用粉色折线实时叠加真实轨迹，并显示 RMS、P95 和最大横向偏差。每次执行生成独立轨迹编号，完整规划点、真实样本与统计结果会原子写入 `data/trajectories/trajectory_*.json`。可用 `RMP_NAV_TRAJECTORY_SAMPLE_INTERVAL` 调整采样间隔，用 `RMP_NAV_TRAJECTORY_MAX_SAMPLES` 限制单次轨迹样本数。

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

## AGX Orin 原生部署

Orin 部署路径固定为 `/mnt/ssd/gt/RobotMapPlanner`，用户级 systemd 单元位于 `deploy/robot-map-planner.service`：

```bash
cd /mnt/ssd/gt/RobotMapPlanner
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/pip install -e '.[test]'
mkdir -p data imports ~/.config/systemd/user
ln -sfn "$PWD/deploy/robot-map-planner.service" ~/.config/systemd/user/robot-map-planner.service
loginctl enable-linger "$USER"
systemctl --user daemon-reload
systemctl --user enable --now robot-map-planner.service
```

服务监听 `0.0.0.0:28200`。使用 `systemctl --user status robot-map-planner.service` 检查状态，使用 `journalctl _SYSTEMD_USER_UNIT=robot-map-planner.service` 查看轮转日志。

## API

- `POST /api/v1/maps/import`
- `GET /api/v1/maps`
- `POST /api/v1/maps/{map_id}/recompile`（从已保存的原始 PCD 按新参数创建独立地图）
- `DELETE /api/v1/maps/{map_id}`
- `POST /api/v1/maps/{map_id}/drafts`
- `PATCH /api/v1/drafts/{draft_id}`
- `POST /api/v1/drafts/{draft_id}/undo|redo|validate|publish`
- `POST /api/v1/versions/{version_id}/activate`
- `GET /api/v1/versions/{version_id}/grid/{layer}`
- `GET /api/v1/drafts/{draft_id}/grid/{layer}`
- `GET /api/v1/versions/{version_id}/tiles/{layer}/{x}/{y}`
- `POST /api/v1/versions/{version_id}/plan`
- `GET /api/v1/navigation/pose`
- `GET /api/v1/navigation/execution`
- `GET /api/v1/navigation/trajectory?after_sequence={sequence}`（增量读取真实轨迹与偏差统计）
- `POST /api/v1/navigation/follow-path`
- `GET /healthz`

API 详细字段可以启动服务后查看 `/docs`。

前端页面的完整操作步骤、参数说明和常见错误处理参见 `docs/FRONTEND_USER_GUIDE.md`。

## 当前边界

- 支持 PCD v0.7 ASCII 和 binary XYZ；明确拒绝 `binary_compressed`。
- 地图和规划仍是静态二维能力；机器人执行仅桥接既有 NavBridge，不实现定位、SLAM、局部控制器、动态避障或完整三维规划。
- 首版为单进程、单用户离线服务，不提供认证或多人协同。
- 规划默认 `max_traversable_cost=0`，禁止进入完整膨胀代价区；可在 API 或规划页面显式提高阈值以启用软代价通行。

## 已验证基线

- `/home/u12297/projects/global_map_20260708_124133.pcd`：243,037 点，生成 `751 x 942` 栅格。
- amd64 原生与 arm64/QEMU 均完成导入、发布和规划；同一输入返回 137 个点，坐标最大差值小于 `1e-12 m`。
- 普通占据编辑只重算变更包围盒及膨胀邻域；边界修改自动回退到完整代价地图编译。
- AGX Orin `aarch64` 原生构建和 pytest 已通过，`http://192.168.1.21:28200` 已完成局域网访问验证。
