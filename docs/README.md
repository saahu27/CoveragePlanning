# Documentation

| Document | Purpose |
|----------|---------|
| [ALGORITHM.md](ALGORITHM.md) | Pseudocode + flowcharts |
| [SEQUENCING_AND_SEGMENTS.md](SEQUENCING_AND_SEGMENTS.md) | Scheduling, splits, replanning |
| [TECHNICAL_REFERENCE.md](TECHNICAL_REFERENCE.md) | Methods-level spec (equations) |

Algorithm flowcharts are PNGs in `images/algorithms/` (embedded in `ALGORITHM.md`). Architecture diagrams are Mermaid sources in `diagrams/`.

---

## Build PDFs

**Recommended:** dev container (has `pandoc`, `xelatex`, `mmdc`, `md-to-pdf`).

```bash
# From repo root inside the dev container:
./docs/verify-docs.sh
```

Outputs: `docs/output/*.pdf` (gitignored).

### Host Ubuntu (without container)

```bash
sudo apt install pandoc lmodern texlive-xetex texlive-latex-extra texlive-fonts-recommended
./docs/verify-docs.sh
```

After changing `docker/Dockerfile`: `docker compose build dev`

### Manual steps

```bash
python3 docs/generate_diagram_pdfs.py --embed-algorithms   # PNG → images/algorithms/
python3 docs/generate_diagram_pdfs.py --all                # architecture PDFs
python3 docs/generate_doc_pdfs.py --all                    # markdown → PDF
```

Check toolchain only:

```bash
python3 docs/generate_diagram_pdfs.py --check
python3 docs/generate_doc_pdfs.py --check
```

---

## Layout

```
docs/
  build-docs.sh              # one-shot build (called by verify-docs.sh)
  verify-docs.sh             # toolchain check + build + assert outputs
  generate_diagram_pdfs.py
  generate_doc_pdfs.py
  diagrams/                  # architecture .mmd
  diagrams/algorithms/       # algorithm .mmd → PNG for ALGORITHM.md
  images/algorithms/         # committed PNGs
  output/                    # generated PDFs (gitignored)
```
