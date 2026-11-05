#!/usr/bin/env bash
set -euo pipefail

source_url="${XGC2_APT_SOURCE_URL:-https://xgc2.apt.xiaokang.ink}"
overlay_url="${XGC2_APT_OVERLAY_URL:-}"
if [[ -z "${XGC2_APT_DISTRIBUTION:-}" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  XGC2_APT_DISTRIBUTION="${VERSION_CODENAME:?OS suite is required}"
fi
distribution="${XGC2_APT_DISTRIBUTION}"
component="${XGC2_APT_COMPONENT:-main}"
list_file="${XGC2_APT_LIST_FILE:-/etc/apt/sources.list.d/xgc2.list}"
arch="$(dpkg --print-architecture)"
case "${distribution}" in
  bionic|focal) ;;
  *) echo "ROS1 utils supports Bionic or Focal, got ${distribution}" >&2; exit 1 ;;
esac

dpkg -s ca-certificates >/dev/null
install -m 0755 -d /etc/apt/keyrings
curl -fsSL --retry 5 https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
  -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
rm -f /etc/apt/sources.list.d/00-xgc2-release-train.list
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s %s %s\n' \
  "${arch}" "${source_url}" "${distribution}" "${component}" > "${list_file}"
if [[ -n "${overlay_url}" ]]; then
  printf 'deb [arch=%s signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s %s %s\n' \
    "${arch}" "${overlay_url%/}" "${distribution}" "${component}" >> "${list_file}"
fi
apt-get update -o Dir::Etc::sourcelist="${list_file}" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
