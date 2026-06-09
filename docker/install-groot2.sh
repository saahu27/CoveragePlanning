#!/usr/bin/env bash
# Groot2 BT editor — x86_64 AppImage only. Optional on arm64/Mac Docker hosts.
set -euo pipefail

TARGETARCH="${TARGETARCH:-$(dpkg --print-architecture 2>/dev/null || uname -m)}"
DEST="${1:-/root/Groot2.AppImage}"

case "${TARGETARCH}" in
  amd64 | x86_64)
    curl -fsSL -o "${DEST}" \
      https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage
    chmod a+x "${DEST}"
    echo "Groot2 installed at ${DEST}"
    ;;
  *)
    echo "Groot2: x86_64 AppImage not available for ${TARGETARCH}; skipping (optional BT editor)."
    exit 0
    ;;
esac
