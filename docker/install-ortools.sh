#!/usr/bin/env bash
# Install Google OR-Tools prebuilt C++ binaries for the Docker build architecture.
#
# amd64  → ubuntu-24.04 tarball (native glibc match)
# arm64  → AlmaLinux-8.10 aarch64 (maintainer-recommended for Ubuntu 24.04 arm64),
#          then debian-11 arm64 fallback. Works on Apple Silicon Docker (linux/arm64).
set -euo pipefail

ORTOOLS_VERSION="${ORTOOLS_VERSION:-9.12.4544}"
ORTOOLS_TAG="${ORTOOLS_TAG:-v9.12}"
TARGETARCH="${TARGETARCH:-$(dpkg --print-architecture 2>/dev/null || uname -m)}"
PREFIX="${ORTOOLS_PREFIX:-/opt/ortools}"
RELEASE_BASE="https://github.com/google/or-tools/releases/download/${ORTOOLS_TAG}"

install_tarball() {
  local url="$1"
  echo "OR-Tools: trying ${url}"
  rm -rf "${PREFIX}"
  mkdir -p "${PREFIX}"
  if ! curl -fSL "${url}" -o /tmp/ortools.tar.gz; then
    return 1
  fi
  if ! tar -xzf /tmp/ortools.tar.gz -C "${PREFIX}" --strip-components=1; then
    rm -f /tmp/ortools.tar.gz
    return 1
  fi
  rm -f /tmp/ortools.tar.gz
  [[ -f "${PREFIX}/lib/cmake/ortools/ortoolsConfig.cmake" ]]
}

case "${TARGETARCH}" in
  amd64 | x86_64)
    URLS=(
      "${RELEASE_BASE}/or-tools_amd64_ubuntu-24.04_cpp_v${ORTOOLS_VERSION}.tar.gz"
    )
    ;;
  arm64 | aarch64)
    URLS=(
      "${RELEASE_BASE}/or-tools_aarch64_AlmaLinux-8.10_cpp_v${ORTOOLS_VERSION}.tar.gz"
      "${RELEASE_BASE}/or-tools_arm64_debian-11_cpp_v${ORTOOLS_VERSION}.tar.gz"
    )
    ;;
  *)
    echo "OR-Tools: unsupported TARGETARCH='${TARGETARCH}'; gpr_planning will use heuristic ATSP." >&2
    exit 0
    ;;
esac

apt-get update
apt-get install -y --no-install-recommends ca-certificates curl
rm -rf /var/lib/apt/lists/*

for url in "${URLS[@]}"; do
  if install_tarball "${url}"; then
    echo "OR-Tools installed at ${PREFIX} (${TARGETARCH})"
    exit 0
  fi
  echo "OR-Tools: install failed for ${url}" >&2
done

echo "OR-Tools: all candidates failed; gpr_planning will use heuristic ATSP." >&2
exit 0
