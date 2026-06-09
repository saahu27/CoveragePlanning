#!/usr/bin/env bash
# Verify doc toolchain and build all PDFs under docs/output/.
# Run from repository root:  ./docs/verify-docs.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${ROOT}/docs/output"

echo "==> Toolchain"
python3 "${ROOT}/docs/generate_diagram_pdfs.py" --check
if ! python3 "${ROOT}/docs/generate_doc_pdfs.py" --check; then
  echo ""
  echo "PDF toolchain incomplete. Install in dev container (recommended):"
  echo "  docker compose build dev"
  echo "  docker compose run --rm dev bash -lc './docs/verify-docs.sh'"
  echo ""
  echo "On host Ubuntu:"
  echo "  sudo apt install pandoc lmodern texlive-xetex texlive-latex-extra texlive-fonts-recommended"
  exit 1
fi
echo ""

echo "==> Build"
bash "${ROOT}/docs/build-docs.sh"
echo ""

echo "==> Verify outputs"
REQUIRED=(
  "ALGORITHM.pdf"
  "SEQUENCING_AND_SEGMENTS.pdf"
  "TECHNICAL_REFERENCE.pdf"
  "software_architecture.pdf"
  "mission_flow_sequence.pdf"
)
missing=0
for name in "${REQUIRED[@]}"; do
  path="${OUT}/${name}"
  if [[ ! -s "${path}" ]]; then
    echo "MISSING or empty: ${path}"
    missing=1
  else
    echo "OK  $(du -h "${path}" | cut -f1)  ${name}"
  fi
done

if [[ "${missing}" -ne 0 ]]; then
  echo ""
  echo "Build incomplete. Use the dev container (pandoc + xelatex + mmdc are preinstalled):"
  echo "  docker compose build dev"
  echo "  docker compose run --rm dev bash -lc './docs/verify-docs.sh'"
  exit 1
fi

echo ""
echo "All required PDFs present in ${OUT}"
