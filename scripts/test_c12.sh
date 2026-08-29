#!/usr/bin/env bash
set -euo pipefail

C12_IP="${1:-192.168.144.108}"
VISIBLE_URL="rtsp://${C12_IP}:554/stream=1"
THERMAL_URL="rtsp://${C12_IP}:555/stream=2"
OUTPUT_DIR="${2:-${HOME}/桌面/C12_验证结果}"
STAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "${OUTPUT_DIR}"

echo "检查 C12 网络连通性：${C12_IP}"
ping -c 2 -W 2 "${C12_IP}"

echo "检查可见光 RTSP：${VISIBLE_URL}"
timeout 15s ffprobe -v error -rtsp_transport tcp \
  -show_entries stream=index,codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 "${VISIBLE_URL}"

echo "检查热成像 RTSP：${THERMAL_URL}"
timeout 15s ffprobe -v error -rtsp_transport tcp \
  -show_entries stream=index,codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 "${THERMAL_URL}"

echo "保存两路截图到：${OUTPUT_DIR}"
timeout 20s ffmpeg -y -rtsp_transport tcp -i "${VISIBLE_URL}" \
  -frames:v 1 "${OUTPUT_DIR}/visible_${STAMP}.png"
timeout 20s ffmpeg -y -rtsp_transport tcp -i "${THERMAL_URL}" \
  -frames:v 1 "${OUTPUT_DIR}/thermal_${STAMP}.png"

echo "C12 两路视频探测与截图完成。"
