#!/usr/bin/env bash
# Build all documentation artifacts under docs/output/.
# Run from repository root:  ./docs/build-docs.sh
# Prefer: ./docs/verify-docs.sh (checks tools and asserts outputs)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${ROOT}/docs/output"
mkdir -p "${OUT}"

echo "==> Algorithm diagrams -> PNG (embedded in ALGORITHM.md)"
python3 "${ROOT}/docs/generate_diagram_pdfs.py" --embed-algorithms

echo "==> Architecture diagrams -> PDF"
python3 "${ROOT}/docs/generate_diagram_pdfs.py" --all --output-dir "${OUT}"

echo "==> Markdown docs -> PDF"
python3 "${ROOT}/docs/generate_doc_pdfs.py" --all --output-dir "${OUT}"

echo ""
echo "Artifacts in ${OUT}:"
ls -1 "${OUT}" 2>/dev/null || true
