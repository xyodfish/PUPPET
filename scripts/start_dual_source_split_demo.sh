#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

RUNTIME_CFG="${1:-${REPO_ROOT}/config/runtime/demo_dual_source_mixed_runtime.yaml}"
VIEWER_CFG="${2:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer.yaml}"

RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
VIEWER_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer"
DIRECT_PASS_SENDER_BIN="${BUILD_DIR}/test/demos/cpp/test_embosa_direct_pass_sender"
SINGLE_CHAIN_IK_SENDER_BIN="${BUILD_DIR}/test/demos/cpp/test_embosa_left_arm_single_chain_ik_sender"

for f in "${RUNTIME_BIN}" "${VIEWER_BIN}" "${DIRECT_PASS_SENDER_BIN}" "${SINGLE_CHAIN_IK_SENDER_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[start_dual_source_split_demo] missing executable: ${f}" >&2
    echo "[start_dual_source_split_demo] build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/dual_source_split_runtime.log"
VIEWER_LOG="${REPO_ROOT}/bin/log/dual_source_split_viewer.log"
DIRECT_PASS_SENDER_LOG="${REPO_ROOT}/bin/log/dual_source_split_direct_pass_sender.log"
SINGLE_CHAIN_IK_SENDER_LOG="${REPO_ROOT}/bin/log/dual_source_split_single_chain_ik_sender.log"

pids=()
cleanup() {
  echo "[start_dual_source_split_demo] stopping nodes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

check_started() {
  local pid="$1"
  local name="$2"
  local log_file="$3"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[start_dual_source_split_demo] ${name} exited during startup, see log: ${log_file}" >&2
    if [[ -f "${log_file}" ]]; then
      sed -n '1,120p' "${log_file}" >&2 || true
    fi
    exit 1
  fi
}

echo "[start_dual_source_split_demo] start runtime"
"${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "runtime" "${RUNTIME_LOG}"

echo "[start_dual_source_split_demo] start viewer"
"${VIEWER_BIN}" "${VIEWER_CFG}" >"${VIEWER_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "viewer" "${VIEWER_LOG}"

echo "[start_dual_source_split_demo] start right-arm sender"
"${DIRECT_PASS_SENDER_BIN}" >"${DIRECT_PASS_SENDER_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "right-arm sender" "${DIRECT_PASS_SENDER_LOG}"

echo "[start_dual_source_split_demo] start left-arm sender"
"${SINGLE_CHAIN_IK_SENDER_BIN}" >"${SINGLE_CHAIN_IK_SENDER_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "left-arm sender" "${SINGLE_CHAIN_IK_SENDER_LOG}"

echo "[start_dual_source_split_demo] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  viewer  log: ${VIEWER_LOG}"
echo "  right-arm sender log: ${DIRECT_PASS_SENDER_LOG}"
echo "  left-arm sender  log: ${SINGLE_CHAIN_IK_SENDER_LOG}"
echo "Press Ctrl+C to stop all."

wait
