#!/bin/bash

# C++代码格式化脚本
# 使用clang-format格式化项目中的C/C++代码，排除第三方和构建产物目录

set -Eeuo pipefail

# 颜色输出定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 获取脚本所在目录，并强制在仓库根执行
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

CHECK_MODE=0
if [[ "${1:-}" == "--check" ]]; then
    CHECK_MODE=1
fi

echo -e "${GREEN}=== C++ Code Formatter ===${NC}"
if [[ ${CHECK_MODE} -eq 1 ]]; then
    echo "Running clang-format in check mode..."
else
    echo "Starting C++ code formatting with clang-format..."
fi

# 检查clang-format是否可用
if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}Error: clang-format is not installed or not in PATH${NC}"
    echo "Please install clang-format first:"
    echo "  Ubuntu/Debian: sudo apt-get install clang-format"
    echo "  CentOS/RHEL:   sudo yum install clang-format"
    echo "  macOS:         brew install clang-format"
    exit 1
fi

# 获取clang-format版本
CLANG_FORMAT_VERSION=$(clang-format --version)
echo -e "${GREEN}Using: ${CLANG_FORMAT_VERSION}${NC}"

# 检查是否存在.clang-format配置文件
if [ -f "${SCRIPT_DIR}/.clang-format" ]; then
    echo -e "${GREEN}Found .clang-format configuration file at: ${SCRIPT_DIR}/.clang-format${NC}"
    STYLE="-style=file"
else
    echo -e "${RED}Error: .clang-format not found in ${SCRIPT_DIR}${NC}"
    echo "Refusing to run with fallback style to avoid accidental large reformat."
    exit 1
fi

# 定义要排除的目录（第三方库路径）
EXCLUDE_DIRS=(
    "third-party"
    "third_party"
    "thirdParty"
    ".git"
    "build"
    "cmake-build-*"
    "__pycache__"
    "*.egg-info"
    "3rd"
    "build-*"
    "install"
    "devel"
    "proto"
)

# 构建find命令的prune参数，避免深入遍历无关目录
PRUNE_EXPR=()
for dir in "${EXCLUDE_DIRS[@]}"; do
    PRUNE_EXPR+=(-path "./${dir}" -o -path "./${dir}/*" -o)
done
# 去掉末尾多余 -o
unset 'PRUNE_EXPR[${#PRUNE_EXPR[@]}-1]'

# 查找所有C++源文件
echo -e "${YELLOW}Searching for C++ files...${NC}"
mapfile -t CPP_FILES < <(find . \
    \( "${PRUNE_EXPR[@]}" \) -prune -o \
    -type f \( \
        -name "*.cpp" -o \
        -name "*.cc" -o \
        -name "*.cxx" -o \
        -name "*.c" -o \
        -name "*.h" -o \
        -name "*.hpp" -o \
        -name "*.hxx" -o \
        -name "*.hh" \
    \) \
    -print)

# 检查是否找到文件
if [ ${#CPP_FILES[@]} -eq 0 ]; then
    echo -e "${YELLOW}No C++ files found to format.${NC}"
    exit 0
fi

echo -e "${GREEN}Found ${#CPP_FILES[@]} C++ files to format${NC}"

# 计数器
PASSED_COUNT=0
FAILED_COUNT=0

# 格式化每个文件
if [[ ${CHECK_MODE} -eq 1 ]]; then
    echo -e "${YELLOW}Checking files...${NC}"
else
    echo -e "${YELLOW}Formatting files...${NC}"
fi

for file in "${CPP_FILES[@]}"; do
    if [[ ${CHECK_MODE} -eq 1 ]]; then
        if clang-format ${STYLE} --dry-run --Werror "$file"; then
            PASSED_COUNT=$((PASSED_COUNT + 1))
            echo -e "${GREEN}✓${NC} OK: $file"
        else
            FAILED_COUNT=$((FAILED_COUNT + 1))
            echo -e "${RED}✗${NC} Needs format: $file"
        fi
    else
        if clang-format ${STYLE} -i "$file"; then
            PASSED_COUNT=$((PASSED_COUNT + 1))
            echo -e "${GREEN}✓${NC} Formatted: $file"
        else
            FAILED_COUNT=$((FAILED_COUNT + 1))
            echo -e "${RED}✗${NC} Failed to format: $file"
        fi
    fi
done

# 输出总结
echo ""
echo -e "${GREEN}=== Formatting Complete ===${NC}"
if [[ ${CHECK_MODE} -eq 1 ]]; then
    echo -e "${GREEN}Files already well-formatted: ${PASSED_COUNT}${NC}"
else
    echo -e "${GREEN}Successfully formatted: ${PASSED_COUNT} files${NC}"
fi

if [ ${FAILED_COUNT} -gt 0 ]; then
    if [[ ${CHECK_MODE} -eq 1 ]]; then
        echo -e "${RED}Files needing format: ${FAILED_COUNT}${NC}"
    else
        echo -e "${RED}Failed to format: ${FAILED_COUNT} files${NC}"
    fi
    exit 1
else
    if [[ ${CHECK_MODE} -eq 1 ]]; then
        echo -e "${GREEN}All files pass format check!${NC}"
    else
        echo -e "${GREEN}All files formatted successfully!${NC}"
    fi
fi

exit 0
