#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

CHANNEL="${1:-zmq}"

case "${CHANNEL}" in
  embosa)
    RUNTIME_CFG="${2:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular.yaml}"
    NODE2_ARG1="${3:-}"
    NODE3_ARG1="${4:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer.yaml}"
    RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
    NODE2_BIN="${BUILD_DIR}/test/demos/cpp/test_embosa_single_chain_ik_sender"
    NODE3_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer"
    SCRIPT_TAG="start_single_chain_ik_modular_3nodes_embosa"
    NODE2_DESC="sender (embosa)"
    NODE3_DESC="viewer (embosa)"
    ;;
  zmq)
    RUNTIME_CFG="${2:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular_zmq.yaml}"
    NODE2_ARG1="${3:-tcp://127.0.0.1:16661}"
    NODE3_ARG1="${4:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer_zmq.yaml}"
    RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
    NODE2_BIN="${BUILD_DIR}/test/demos/cpp/test_zmq_single_chain_ik_sender"
    NODE3_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer_zmq"
    SCRIPT_TAG="start_single_chain_ik_modular_3nodes_zmq"
    NODE2_DESC="sender (zmq)"
    NODE3_DESC="viewer (zmq)"
    ;;
  tcp)
    RUNTIME_CFG="${2:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular_tcp.yaml}"
    NODE2_ARG1="${3:-0.0.0.0}"
    NODE3_ARG1="${4:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer_tcp.yaml}"
    RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
    NODE2_BIN="${BUILD_DIR}/test/demos/cpp/test_tcp_single_chain_ik_sender"
    NODE3_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer_tcp"
    SCRIPT_TAG="start_single_chain_ik_modular_3nodes_tcp"
    NODE2_DESC="sender (tcp)"
    NODE3_DESC="viewer (tcp)"
    ;;
  udp)
    RUNTIME_CFG="${2:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular_udp.yaml}"
    NODE2_ARG1="${3:-127.0.0.1}"
    NODE3_ARG1="${4:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer_udp.yaml}"
    RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
    NODE2_BIN="${BUILD_DIR}/test/demos/cpp/test_udp_single_chain_ik_sender"
    NODE3_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer_udp"
    SCRIPT_TAG="start_single_chain_ik_modular_3nodes_udp"
    NODE2_DESC="sender (udp)"
    NODE3_DESC="viewer (udp)"
    ;;
  *)
    echo "Usage: $0 <channel> [runtime_cfg] [node2_arg] [node3_arg]" >&2
    echo "channel: embosa | zmq | tcp | udp" >&2
    exit 1
    ;;
esac

for f in "${RUNTIME_BIN}" "${NODE2_BIN}" "${NODE3_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[${SCRIPT_TAG}] missing executable: ${f}" >&2
    echo "Build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/${SCRIPT_TAG}_runtime.log"
NODE2_LOG="${REPO_ROOT}/bin/log/${SCRIPT_TAG}_node2.log"
NODE3_LOG="${REPO_ROOT}/bin/log/${SCRIPT_TAG}_node3.log"

pids=()
cleanup() {
  echo "[${SCRIPT_TAG}] stopping nodes..."
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
    echo "[${SCRIPT_TAG}] ${name} exited during startup, see log: ${log_file}" >&2
    if [[ -f "${log_file}" ]]; then
      sed -n '1,120p' "${log_file}" >&2 || true
    fi
    exit 1
  fi
}

wait_tcp_listener() {
  local port="$1"
  local name="$2"
  local max_retry=30
  local retry=0
  while (( retry < max_retry )); do
    local found=0
    while IFS= read -r line; do
      if [[ "${line}" == *":${port} "* ]]; then
        found=1
        break
      fi
    done < <(ss -ltn 2>/dev/null || true)
    if (( found == 1 )); then
      return 0
    fi
    sleep 0.2
    ((retry++))
  done
  echo "[${SCRIPT_TAG}] ${name} did not listen on tcp port ${port} in time" >&2
  return 1
}

start_node2() {
  if [[ "${CHANNEL}" == "embosa" ]]; then
    "${NODE2_BIN}" >"${NODE2_LOG}" 2>&1 &
  elif [[ "${CHANNEL}" == "zmq" ]]; then
    "${NODE2_BIN}" "${NODE2_ARG1}" "puppet_demo/primitive_frame" >"${NODE2_LOG}" 2>&1 &
  elif [[ "${CHANNEL}" == "tcp" || "${CHANNEL}" == "udp" ]]; then
    "${NODE2_BIN}" "${NODE2_ARG1}" "16661" >"${NODE2_LOG}" 2>&1 &
  fi
  pids+=("$!")
  sleep 1
  check_started "${pids[$((${#pids[@]} - 1))]}" "${NODE2_DESC}" "${NODE2_LOG}"
}

start_node3() {
  "${NODE3_BIN}" "${NODE3_ARG1}" >"${NODE3_LOG}" 2>&1 &
  pids+=("$!")
  sleep 1
  check_started "${pids[$((${#pids[@]} - 1))]}" "${NODE3_DESC}" "${NODE3_LOG}"
}

start_runtime() {
  "${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
  pids+=("$!")
  sleep 1
  check_started "${pids[$((${#pids[@]} - 1))]}" "runtime" "${RUNTIME_LOG}"
}

if [[ "${CHANNEL}" == "tcp" ]]; then
  # TCP runtime connects out to sender/viewer sockets, so start them first.
  echo "[${SCRIPT_TAG}] start node2 ${NODE2_DESC}"
  start_node2
  wait_tcp_listener 16661 "${NODE2_DESC}" || exit 1
  echo "[${SCRIPT_TAG}] start node3 ${NODE3_DESC}"
  start_node3
  wait_tcp_listener 16663 "${NODE3_DESC}" || exit 1
  echo "[${SCRIPT_TAG}] start node1 runtime"
  start_runtime
else
  echo "[${SCRIPT_TAG}] start node1 runtime"
  start_runtime
  echo "[${SCRIPT_TAG}] start node2 ${NODE2_DESC}"
  start_node2
  echo "[${SCRIPT_TAG}] start node3 ${NODE3_DESC}"
  start_node3
fi

echo "[${SCRIPT_TAG}] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  node2   log: ${NODE2_LOG}"
echo "  node3   log: ${NODE3_LOG}"
echo "Press Ctrl+C to stop all."

wait
