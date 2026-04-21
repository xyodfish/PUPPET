#!/usr/bin/env bash
set -euo pipefail

if [ -t 1 ]; then
  YELLOW='\033[1;33m'
  RED='\033[1;31m'
  NC='\033[0m'
else
  YELLOW='' ; RED='' ; NC=''
fi

if cmake -S . -B build && cmake --build build -j"$(nproc)"; then
  echo -e "${YELLOW}[PUPPET] build finished successfully.${NC}"
else
  echo -e "${RED}[PUPPET] build failed.${NC}"
  exit 1
fi
"$(cd "$(dirname "$0")" && pwd)/scripts/print_puppet_logo.sh"
