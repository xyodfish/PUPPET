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

renderLine() {
  local line="$1"
  line="${line//\{G\}/$GREEN}"
  line="${line//\{C\}/$CYAN}"
  line="${line//\{B\}/$BLUE}"
  line="${line//\{Y\}/$YELLOW}"
  line="${line//\{M\}/$MAGENTA}"
  line="${line//\{W\}/$WHITE}"
  line="${line//\{N\}/$NC}"
  printf '%b\n' "$line"
}

lines=(
  "{G}╔══════════════════════════════════════════════════════════════════════════╗{N}"
  "{G}║{N} {C}[ VR / MOCAP ]{N} {G}=>{N} {Y}[ PRIMITIVE FRAME ]{N} {G}=>{N} {B}[ RETARGET / WBC ]{N} {G}=>{N} {M}[ ROBOT ]{N} {G}║{N}"
  "{G}╠══════════════════════════════════════════════════════════════════════════╣{N}"
  "{G}║{N}                                                                          {G}║{N}"
  "{G}║{N}        {G}____   __  __   ____   ____   _____   ______{N}                      {G}║{N}"
  "{G}║{N}       {C}/ __ \\\\ / / / /  / __ \\\\ / __ \\\\ / ____| /_  __/{N}                      {G}║{N}"
  "{G}║{N}      {B}/ /_/ // / / /  / /_/ // /_/ // __/    / /{N}                          {G}║{N}"
  "{G}║{N}     {M}/ ____// /_/ /  / ____// ____// /___   / /{N}                           {G}║{N}"
  "{G}║{N}    {Y}/_/     \\\\____/  /_/    /_/    /_____/  /_/{N}                            {G}║{N}"
  "{G}║{N}                                                                          {G}║{N}"
  "{G}║{N}           {B}Primitive-based Embodied Teleoperation Platform{N}                {G}║{N}"
  "{G}║{N}               {C}[ modular / pluggable / unified ]{N}                          {G}║{N}"
  "{G}║{N}                                                                          {G}║{N}"
  "{G}╚══════════════════════════════════════════════════════════════════════════╝{N}"
)

for line in "${lines[@]}"; do
  renderLine "$line"
done