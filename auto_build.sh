#!/usr/bin/env bash
set -euo pipefail

help_function() {
  cat <<'EOF'
===== PUPPET Auto Build =====
用法: ./auto_build.sh [选项]

选项:
  --all            构建主工程 + proto（默认）
  --proto          在当前构建集合中包含 proto 构建
  --proto-only     仅构建 proto
  --main-only      仅构建主工程
  --install-proto  构建 proto 后执行安装
  --install-prefix 指定 proto 安装前缀（默认: ./devel/x86_64-Linux-GNU-9.4.0）
  --cmake-install-dir 指定 proto 的 CMake package 安装相对目录
                     （默认: lib/cmake/puppet_proto）
  -j, --jobs N     并行编译线程数（默认: nproc）
  --clean          构建前清理 build 与 proto/build（全量清理）
  --no-clean-proto 禁用 proto 构建时的自动清理（用于快速迭代调试）
  -h, --help       显示帮助

说明:
  - Proto 构建时会自动清理 proto/build 目录，确保生成文件版本一致
  - 如需禁用此行为（快速调试），可使用 --no-clean-proto 选项

示例:
  ./auto_build.sh
  ./auto_build.sh --proto-only
  ./auto_build.sh --proto-only --install-proto --install-prefix ./devel/x86_64-Linux-GNU-9.4.0
  ./auto_build.sh --proto-only --cmake-install-dir lib/cmake/puppet_proto_v2
  ./auto_build.sh --main-only -j 12
  ./auto_build.sh --clean --proto
  ./auto_build.sh --proto-only --no-clean-proto  # 快速调试，不清理
EOF
}

CURRENT_DIR="$(cd "$(dirname "$0")" && pwd)"
JOBS="$(nproc)"
CLEAN_FIRST=false
CLEAN_PROTO=true  # 默认启用 proto 自动清理
BUILD_MAIN=true
BUILD_PROTO=true
INSTALL_PROTO=false
INSTALL_PREFIX="${CURRENT_DIR}/devel/x86_64-Linux-GNU-9.4.0"
DEFAULT_PROTO_CMAKE_INSTALL_DIR="lib/cmake/puppet_proto"
PROTO_CMAKE_INSTALL_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      BUILD_MAIN=true
      BUILD_PROTO=true
      shift
      ;;
    --proto)
      BUILD_PROTO=true
      shift
      ;;
    --proto-only)
      BUILD_MAIN=false
      BUILD_PROTO=true
      shift
      ;;
    --main-only)
      BUILD_MAIN=true
      BUILD_PROTO=false
      shift
      ;;
    --install-proto)
      INSTALL_PROTO=true
      BUILD_PROTO=true
      shift
      ;;
    --install-prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --install-prefix=*)
      INSTALL_PREFIX="${1#--install-prefix=}"
      shift
      ;;
    --cmake-install-dir)
      PROTO_CMAKE_INSTALL_DIR="$2"
      shift 2
      ;;
    --cmake-install-dir=*)
      PROTO_CMAKE_INSTALL_DIR="${1#--cmake-install-dir=}"
      shift
      ;;
    -j|--jobs)
      JOBS="$2"
      shift 2
      ;;
    -j[0-9]*)
      JOBS="${1#-j}"
      shift
      ;;
    --clean)
      CLEAN_FIRST=true
      shift
      ;;
    --no-clean-proto)
      CLEAN_PROTO=false
      shift
      ;;
    -h|--help)
      help_function
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      help_function
      exit 1
      ;;
  esac
done

if [[ "${CLEAN_FIRST}" == "true" ]]; then
  echo "[auto_build] clean build directories..."
  rm -rf "${CURRENT_DIR}/build" "${CURRENT_DIR}/proto/build"
fi

build_main() {
  echo "[auto_build] configure main project..."
  cmake -S "${CURRENT_DIR}" -B "${CURRENT_DIR}/build"
  echo "[auto_build] build main project (-j${JOBS})..."
  cmake --build "${CURRENT_DIR}/build" -j"${JOBS}"
}

build_proto() {
  # 默认清理 proto/build 目录，确保生成文件版本一致
  if [[ "${CLEAN_PROTO}" == "true" ]]; then
    echo "[auto_build] cleaning proto/build directory to ensure consistency..."
    rm -rf "${CURRENT_DIR}/proto/build"
  else
    echo "[auto_build] skipping proto/build cleanup (fast mode)"
  fi
  
  echo "[auto_build] configure proto project..."
  effectiveProtoCmakeInstallDir="${PROTO_CMAKE_INSTALL_DIR:-${DEFAULT_PROTO_CMAKE_INSTALL_DIR}}"
  cmakeArgs=(-S "${CURRENT_DIR}/proto" -B "${CURRENT_DIR}/proto/build")
  cmakeArgs+=("-DPUPPET_PROTO_CMAKE_INSTALL_DIR=${effectiveProtoCmakeInstallDir}")
  cmake "${cmakeArgs[@]}"
  echo "[auto_build] build proto project (-j${JOBS})..."
  cmake --build "${CURRENT_DIR}/proto/build" -j"${JOBS}"

  if [[ "${INSTALL_PROTO}" == "true" ]]; then
    echo "[auto_build] install proto -> ${INSTALL_PREFIX}"
    cmake --install "${CURRENT_DIR}/proto/build" --prefix "${INSTALL_PREFIX}"
  fi
}

echo "[auto_build] settings: BUILD_MAIN=${BUILD_MAIN}, BUILD_PROTO=${BUILD_PROTO}, INSTALL_PROTO=${INSTALL_PROTO}, INSTALL_PREFIX=${INSTALL_PREFIX}, PROTO_CMAKE_INSTALL_DIR=${PROTO_CMAKE_INSTALL_DIR:-${DEFAULT_PROTO_CMAKE_INSTALL_DIR}}, JOBS=${JOBS}, CLEAN_PROTO=${CLEAN_PROTO}"

if [[ "${BUILD_MAIN}" == "true" ]]; then
  build_main
fi

if [[ "${BUILD_PROTO}" == "true" ]]; then
  build_proto
fi

echo "[auto_build] done."
