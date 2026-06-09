#!/usr/bin/env bash
# Headless browser for mermaid-cli (mmdc). Arch-specific install, no hardcoded amd64 .deb.
set -euo pipefail

TARGETARCH="${TARGETARCH:-$(dpkg --print-architecture 2>/dev/null || uname -m)}"

apt-get update
apt-get install -y --no-install-recommends wget ca-certificates fonts-liberation

case "${TARGETARCH}" in
  amd64 | x86_64)
    wget -q -O /tmp/google-chrome.deb \
      https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
    apt-get install -y /tmp/google-chrome.deb
    rm -f /tmp/google-chrome.deb
    BROWSER="/usr/bin/google-chrome-stable"
    ;;
  arm64 | aarch64)
    # Google Chrome .deb is amd64-only; Chromium from apt works in arm64 containers.
    apt-get install -y --no-install-recommends chromium-browser || \
      apt-get install -y --no-install-recommends chromium
    if [[ -x /usr/bin/chromium ]]; then
      BROWSER="/usr/bin/chromium"
    elif [[ -x /usr/bin/chromium-browser ]]; then
      BROWSER="/usr/bin/chromium-browser"
    else
      echo "Chromium not found after apt install on ${TARGETARCH}" >&2
      exit 1
    fi
    ;;
  *)
    echo "Unsupported TARGETARCH='${TARGETARCH}' for Chromium install." >&2
    exit 1
    ;;
esac

if [[ ! -x "${BROWSER}" ]]; then
  echo "Browser not executable: ${BROWSER}" >&2
  exit 1
fi

mkdir -p /etc/profile.d
cat >/etc/profile.d/gpr-puppeteer.sh <<EOF
export PUPPETEER_EXECUTABLE_PATH=${BROWSER}
export PUPPETEER_SKIP_CHROMIUM_DOWNLOAD=true
EOF
chmod 644 /etc/profile.d/gpr-puppeteer.sh
echo "Puppeteer browser: ${BROWSER} (${TARGETARCH})"
