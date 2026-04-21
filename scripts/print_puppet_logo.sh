#!/usr/bin/env bash
set -euo pipefail

if [ -t 1 ]; then
  GREEN='\033[1;32m'
  CYAN='\033[1;36m'
  BLUE='\033[1;34m'
  YELLOW='\033[1;33m'
  MAGENTA='\033[1;35m'
  WHITE='\033[1;37m'
  NC='\033[0m'
else
  GREEN='' ; CYAN='' ; BLUE='' ; YELLOW='' ; MAGENTA='' ; WHITE='' ; NC=''
fi

echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║${NC} ${CYAN}[ VR / MOCAP ]${NC} ${GREEN}=>${NC} ${YELLOW}[ PRIMITIVE FRAME ]${NC} ${GREEN}=>${NC} ${BLUE}[ RETARGET / WBC ]${NC} ${GREEN}=>${NC} ${MAGENTA}[ ROBOT ]${NC} ${GREEN}║${NC}"
echo -e "${GREEN}╠══════════════════════════════════════════════════════════════════════════╣${NC}"
echo -e "${GREEN}║${NC}                                                                          ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}        ${GREEN}____   __  __   ____   ____   _____   ______${NC}                      ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}       ${CYAN}/ __ \\\\ / / / /  / __ \\\\ / __ \\\\ / ____| /_  __/${NC}                      ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}      ${BLUE}/ /_/ // / / /  / /_/ // /_/ // __/    / /${NC}                          ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}     ${MAGENTA}/ ____// /_/ /  / ____// ____// /___   / /${NC}                           ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}    ${YELLOW}/_/     \\\\____/  /_/    /_/    /_____/  /_/${NC}                            ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}                                                                          ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}           ${BLUE}Primitive-based Embodied Teleoperation Platform${NC}                ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}               ${CYAN}[ modular / pluggable / unified ]${NC}                          ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}                                                                          ${GREEN}║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════════════╝${NC}"
