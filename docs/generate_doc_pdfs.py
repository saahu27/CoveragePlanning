#!/usr/bin/env python3
"""Export Markdown documentation under docs/ to PDF.

Tries backends in order:
  1. pandoc + xelatex (best math rendering for TECHNICAL_REFERENCE.md)
  2. pandoc + wkhtmltopdf
  3. npx md-to-pdf (Node; no LaTeX required)

Examples:
  python3 docs/generate_doc_pdfs.py --all
  python3 docs/generate_doc_pdfs.py docs/TECHNICAL_REFERENCE.md
  python3 docs/generate_doc_pdfs.py --all --backend pandoc
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

DOCS_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = DOCS_DIR / "output"

DEFAULT_DOC_STEMS = (
    "ALGORITHM",
    "SEQUENCING_AND_SEGMENTS",
    "TECHNICAL_REFERENCE",
)


def find_pandoc() -> str | None:
    return shutil.which("pandoc")


def find_xelatex() -> str | None:
    return shutil.which("xelatex")


def find_wkhtmltopdf() -> str | None:
    return shutil.which("wkhtmltopdf")


def find_npx() -> str | None:
    return shutil.which("npx")


def find_md_to_pdf() -> str | None:
    return shutil.which("md-to-pdf")


def collect_inputs(stems: list[str], render_all: bool) -> list[Path]:
    if render_all:
        inputs = [DOCS_DIR / f"{stem}.md" for stem in DEFAULT_DOC_STEMS]
        missing = [p for p in inputs if not p.is_file()]
        if missing:
            sys.exit("missing doc files: " + ", ".join(str(p) for p in missing))
        return inputs

    if not stems:
        sys.exit("provide .md paths or use --all")

    resolved: list[Path] = []
    for raw in stems:
        path = Path(raw)
        if not path.is_absolute():
            path = Path.cwd() / path
        if not path.is_file():
            sys.exit(f"input not found: {path}")
        if path.suffix != ".md":
            sys.exit(f"expected .md file: {path}")
        resolved.append(path.resolve())
    return resolved


def run(cmd: list[str], *, dry_run: bool) -> None:
    print("  ", " ".join(cmd))
    if dry_run:
        return
    subprocess.run(cmd, check=True)


def run_optional(cmd: list[str], *, dry_run: bool) -> bool:
    print("  ", " ".join(cmd))
    if dry_run:
        return True
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        return False
    return True


def export_pandoc_xelatex(md: Path, pdf: Path, *, dry_run: bool) -> bool:
    if find_pandoc() is None or find_xelatex() is None:
        return False
    return run_optional(
        [
            find_pandoc() or "pandoc",
            str(md),
            "-o",
            str(pdf),
            "--pdf-engine=xelatex",
            "--resource-path",
            str(DOCS_DIR),
            "-V",
            "geometry:margin=2.5cm",
            "-V",
            "documentclass=article",
            "-V",
            "fontsize=11pt",
            "--highlight-style=tango",
            "--toc",
            "--toc-depth=2",
        ],
        dry_run=dry_run,
    )


def export_pandoc_wkhtml(md: Path, pdf: Path, *, dry_run: bool) -> bool:
    if find_pandoc() is None or find_wkhtmltopdf() is None:
        return False
    run(
        [
            find_pandoc() or "pandoc",
            str(md),
            "-o",
            str(pdf),
            "--pdf-engine=wkhtmltopdf",
            "--highlight-style=tango",
        ],
        dry_run=dry_run,
    )
    return True


def export_md_to_pdf(md: Path, pdf: Path, *, dry_run: bool) -> bool:
    exe = find_md_to_pdf()
    cmd: list[str]
    if exe is not None:
        cmd = [exe]
    elif find_npx() is not None:
        cmd = [find_npx() or "npx", "--yes", "md-to-pdf@5.2.4"]
    else:
        return False

    # md-to-pdf writes beside the .md by default; move into docs/output/ afterward.
    if not run_optional(
        [
            *cmd,
            str(md),
            "--basedir",
            str(DOCS_DIR),
        ],
        dry_run=dry_run,
    ):
        return False

    if dry_run:
        return True

    sidecar = md.with_suffix(".pdf")
    if sidecar.resolve() != pdf.resolve():
        if not sidecar.is_file():
            return False
        pdf.parent.mkdir(parents=True, exist_ok=True)
        sidecar.replace(pdf)
    return pdf.is_file()


def export_one(
    md: Path,
    output_dir: Path,
    backend: str,
    *,
    dry_run: bool,
) -> None:
    pdf = output_dir / f"{md.stem}.pdf"
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"export {md.name} -> {pdf}")

    if backend == "pandoc":
        if export_pandoc_xelatex(md, pdf, dry_run=dry_run):
            return
        if export_pandoc_wkhtml(md, pdf, dry_run=dry_run):
            return
        sys.exit(
            "pandoc backend requires pandoc and xelatex (or wkhtmltopdf). "
            "Try: sudo apt install pandoc texlive-xetex"
        )

    if backend == "md-to-pdf":
        if export_md_to_pdf(md, pdf, dry_run=dry_run):
            return
        sys.exit("md-to-pdf backend requires npx (Node.js).")

    # auto
    if export_pandoc_xelatex(md, pdf, dry_run=dry_run):
        return
    if export_md_to_pdf(md, pdf, dry_run=dry_run):
        return
    if export_pandoc_wkhtml(md, pdf, dry_run=dry_run):
        return

    sys.exit(
        "no PDF backend available. Install one of:\n"
        "  sudo apt install pandoc texlive-xetex          # recommended (dev container)\n"
        "  npm install -g md-to-pdf                       # fallback (needs Chromium)\n"
        "Or run inside the dev container: ./docs/build-docs.sh\n"
    )


def backends_available() -> bool:
    return bool((find_pandoc() and find_xelatex()) or find_md_to_pdf())


def check_backends() -> None:
    """Print which PDF backends are available (for verify-docs.sh)."""
    pandoc = find_pandoc()
    xelatex = find_xelatex()
    md_to_pdf = find_md_to_pdf()
    npx = find_npx()
    print("PDF backends:")
    print(f"  pandoc:     {pandoc or 'missing'}")
    print(f"  xelatex:    {xelatex or 'missing'}")
    print(f"  md-to-pdf:  {md_to_pdf or 'missing'}")
    print(f"  npx:        {npx or 'missing'}")
    if pandoc and xelatex:
        print("  -> will use pandoc + xelatex")
    elif md_to_pdf:
        print("  -> will use md-to-pdf (fallback)")
    else:
        print("  -> none ready; install pandoc + lmodern + texlive-xetex")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export docs/*.md to PDF.")
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="markdown files (omit with --all)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help=f"export {', '.join(DEFAULT_DOC_STEMS)}.md",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="output directory (default: docs/output)",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "pandoc", "md-to-pdf"),
        default="auto",
        help="PDF tool chain (default: auto)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print commands only",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="print available PDF backends and exit",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.check:
        check_backends()
        if not backends_available():
            sys.exit(1)
        return
    if args.all:
        inputs = collect_inputs([], True)
    else:
        inputs = collect_inputs([str(p) for p in args.inputs], False)

    for md in inputs:
        export_one(
            md,
            args.output_dir.resolve(),
            args.backend,
            dry_run=args.dry_run,
        )

    print(f"done ({len(inputs)} document(s))")


if __name__ == "__main__":
    main()
