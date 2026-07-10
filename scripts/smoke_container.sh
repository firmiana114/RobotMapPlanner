#!/usr/bin/env bash
set -euo pipefail

IMAGE="${RMP_IMAGE:-robot-map-planner:0.1.0}"
NAME="rmp-smoke-${RANDOM}"
cleanup() { docker rm -f "${NAME}" >/dev/null 2>&1 || true; }
trap cleanup EXIT

echo "INFO 启动容器冒烟测试 image=${IMAGE}"
docker run -d --name "${NAME}" --network host "${IMAGE}" >/dev/null
for _ in $(seq 1 30); do
  if docker exec "${NAME}" python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:28200/healthz', timeout=2)"; then
    echo "INFO 容器健康检查通过"
    exit 0
  fi
  sleep 1
done
echo "ERROR 容器健康检查超时" >&2
docker logs "${NAME}" >&2
exit 1
