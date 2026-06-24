#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
VERSION="${PACKAGE_VERSION:-1.0.1-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PACKAGE="ros-noetic-xgc2-ros1-utils"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}/${PACKAGE}_"*.deb

pkg_root="${BUILD_DIR}/${PACKAGE}"
mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${PACKAGE}"

copy_path() {
  local src="$1"
  if [[ -e "${src}" ]]; then
    mkdir -p "${pkg_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${pkg_root}${src#${INSTALL_ROOT}}"
  fi
}

copy_path "${PREFIX_ROOT}/share/ros1_utils"
copy_path "${PREFIX_ROOT}/include/ros1_utils"
copy_path "${PREFIX_ROOT}/include/controller_runtime"
copy_path "${PREFIX_ROOT}/include/control_utils"
copy_path "${PREFIX_ROOT}/lib/libros1_utils_ugv_identification.so"

if [[ ! -d "${pkg_root}${PREFIX}/share/ros1_utils" ]]; then
  echo "missing installed ros1_utils share directory" >&2
  exit 1
fi
if [[ ! -f "${pkg_root}${PREFIX}/lib/libros1_utils_ugv_identification.so" ]]; then
  echo "missing installed ros1_utils ugv identification library" >&2
  exit 1
fi

cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: libceres-dev, libeigen3-dev, libxgc2-math-dev (>= 0.4.0-1), ros-noetic-roscpp
Provides: ros-noetic-controller-runtime, ros-noetic-control-utils
Conflicts: ros-noetic-controller-runtime, ros-noetic-control-utils
Description: XGC2 ROS1 runtime and control utilities
 Header and library package for ROS1-coupled controller runtime,
 scheduling, topic helpers, and control utility code.
EOF

printf '%s package\n' "${PACKAGE}" > "${pkg_root}/usr/share/doc/${PACKAGE}/README"
find "${pkg_root}" -type d -exec chmod 0755 {} +
find "${pkg_root}" -type f -exec chmod 0644 {} +
chmod 0755 "${pkg_root}/DEBIAN"
fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "${PACKAGE}_*.deb" -print | sort
