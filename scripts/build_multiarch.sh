#!/usr/bin/env bash
set -euo pipefail

IMAGE="${RMP_IMAGE:-robot-map-planner:0.1.0}"
BUILDER="${RMP_BUILDER:-robot-map-planner-builder}"
PLATFORMS="${RMP_PLATFORMS:-linux/amd64,linux/arm64}"
OUTPUT_MODE="${RMP_OUTPUT_MODE:-load}"

if ! docker buildx inspect "${BUILDER}" >/dev/null 2>&1; then
  docker buildx create --name "${BUILDER}" --use
else
  docker buildx use "${BUILDER}"
fi

output_flag="--load"
if [[ "${OUTPUT_MODE}" == "push" ]]; then
  output_flag="--push"
elif [[ "${PLATFORMS}" == *,* ]]; then
  echo "INFO 多平台镜像不能同时 --load；将改为仅构建缓存。设置 RMP_OUTPUT_MODE=push 可推送 manifest。"
  output_flag="--output=type=cacheonly"
fi

echo "INFO 构建 RobotMapPlanner 镜像 image=${IMAGE} platforms=${PLATFORMS} output=${OUTPUT_MODE}"
docker buildx build --platform "${PLATFORMS}" -t "${IMAGE}" ${output_flag} .
