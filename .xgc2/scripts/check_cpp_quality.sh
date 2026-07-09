#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/cpp-quality-catkin}"
BUILD_DIR="${WORK_DIR}/build"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      BUILD_DIR="${WORK_DIR}/build"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

require_tool() {
  local tool="$1"
  if ! command -v "${tool}" >/dev/null; then
    echo "missing required tool: ${tool}" >&2
    exit 1
  fi
}

require_tool clang-format
require_tool clang-tidy
require_tool catkin_make
require_tool git
require_tool rsync

git config --global --add safe.directory "${REPO_ROOT}" >/dev/null 2>&1 || true

mapfile -t FORMAT_FILES < <(
  cd "${REPO_ROOT}"
  git ls-files '*.cpp' '*.h' '*.hpp' | while IFS= read -r file; do
    [[ -e "${file}" ]] && printf '%s\n' "${file}"
  done | sort
)

if [[ "${#FORMAT_FILES[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

(
  cd "${REPO_ROOT}"
  clang-format --dry-run --Werror "${FORMAT_FILES[@]}"
)

rm -rf "${WORK_DIR}/src" "${BUILD_DIR}" "${WORK_DIR}/devel"
mkdir -p "${WORK_DIR}/src/ros1_utils"
rsync -a --delete \
  --exclude .git \
  --exclude .work \
  --exclude debs \
  "${REPO_ROOT}/" "${WORK_DIR}/src/ros1_utils/"

if [[ -f /opt/ros/melodic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
fi

cd "${WORK_DIR}"
catkin_make \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-O2 -g -DNDEBUG" \
  -DCMAKE_C_FLAGS_RELWITHDEBINFO="-O2 -g -DNDEBUG"

mapfile -t TIDY_FILES < <(
  cd "${REPO_ROOT}"
  git ls-files 'src/*.cpp' 'test/*.cpp' | while IFS= read -r file; do
    [[ -e "${file}" ]] && printf '%s\n' "${file}"
  done | sort
)

for file in "${TIDY_FILES[@]}"; do
  clang-tidy --quiet "${WORK_DIR}/src/ros1_utils/${file}" -p "${BUILD_DIR}"
done

echo "C++ quality check passed"
