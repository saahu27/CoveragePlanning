#!/usr/bin/env python3
"""Render Mermaid .mmd sources under docs/diagrams/ to PDF (or PNG/SVG).

Requires @mermaid-js/mermaid-cli (mmdc) and Chromium. Both are installed in the
dev Docker image (see docker/Dockerfile).

Examples:
  python3 docs/generate_diagram_pdfs.py --all
  python3 docs/generate_diagram_pdfs.py docs/diagrams/software_architecture.mmd
  python3 docs/generate_diagram_pdfs.py --all --format png
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

DOCS_DIR = Path(__file__).resolve().parent
DEFAULT_DIAGRAMS_DIR = DOCS_DIR / "diagrams"
ALGORITHMS_DIAGRAMS_DIR = DEFAULT_DIAGRAMS_DIR / "algorithms"
EMBEDDED_ALGORITHM_IMAGES_DIR = DOCS_DIR / "images" / "algorithms"
DEFAULT_OUTPUT_DIR = DOCS_DIR / "output"
DEFAULT_PUPPETEER_CONFIG = DOCS_DIR / "mermaid-puppeteer.json"

# Per-diagram render hints for large architecture / sequence charts.
RENDER_HINTS: dict[str, dict[str, int]] = {
    "software_architecture.mmd": {"width": 2600},
    "mission_flow_sequence.mmd": {"width": 2000, "height": 9000},
    "mission_loop.mmd": {"width": 1500, "height": 1100},
    "perception_pipeline.mmd": {"width": 1700, "height": 900},
    "segment_split.mmd": {"width": 1400, "height": 1200},
    "recompute_schedule.mmd": {"width": 1600, "height": 2000},
    "execute_job.mmd": {"width": 1500, "height": 2200},
}


def resolve_mmdc_argv(*, dry_run: bool) -> list[str]:
    """Return argv prefix to invoke mermaid-cli (mmdc).

    Resolution order: `mmdc` on PATH (dev Docker / global install), then
    `npx --yes @mermaid-js/mermaid-cli` (downloads on first use; no repo install).
    """
    if dry_run:
        return ["mmdc"]

    mmdc = shutil.which("mmdc")
    if mmdc is not None:
        return [mmdc]

    local_prefix = Path.home() / ".local" / "share" / "gpr-mmdc" / "node_modules" / ".bin" / "mmdc"
    if local_prefix.is_file():
        return [str(local_prefix)]

    npx = shutil.which("npx")
    if npx is not None:
        return [npx, "--yes", "@mermaid-js/mermaid-cli"]

    sys.exit(
        "mmdc not found. Need one of:\n"
        "  - mmdc on PATH (dev Docker image has it)\n"
        "  - Node.js npx (auto-fetches @mermaid-js/mermaid-cli)\n"
        "  - npm install -g @mermaid-js/mermaid-cli\n"
        "  - npm install --prefix ~/.local/share/gpr-mmdc @mermaid-js/mermaid-cli"
    )


def find_chromium() -> str | None:
    """Resolve a Chromium/Chrome binary for Puppeteer (mmdc)."""
    configured = os.environ.get("PUPPETEER_EXECUTABLE_PATH")
    if configured and Path(configured).is_file():
        return configured

    candidates = [
        "/usr/bin/google-chrome-stable",
        "/usr/bin/google-chrome",
        "/usr/bin/chromium",
        "/usr/bin/chromium-browser",
    ]
    for candidate in candidates:
        if Path(candidate).is_file():
            # Ubuntu chromium-browser is often a snap stub; skip if not executable.
            if candidate.endswith("chromium-browser"):
                try:
                    with open(candidate, encoding="utf-8", errors="ignore") as handle:
                        if "snap install chromium" in handle.read(4096):
                            continue
                except OSError:
                    continue
            return candidate
    return shutil.which("google-chrome-stable") or shutil.which("google-chrome") or shutil.which(
        "chromium"
    )


def mmdc_env() -> dict[str, str]:
    env = os.environ.copy()
    chromium = find_chromium()
    if chromium:
        env["PUPPETEER_EXECUTABLE_PATH"] = chromium
        env.setdefault("PUPPETEER_SKIP_CHROMIUM_DOWNLOAD", "true")
    return env


def collect_inputs(paths: list[Path], diagrams_dir: Path, render_all: bool) -> list[Path]:
    if render_all:
        if not diagrams_dir.is_dir():
            sys.exit(f"diagrams directory not found: {diagrams_dir}")
        inputs = sorted(diagrams_dir.glob("*.mmd"))
        if not inputs:
            sys.exit(f"no .mmd files in {diagrams_dir}")
        return inputs

    if not paths:
        sys.exit("provide .mmd file paths or use --all")

    resolved: list[Path] = []
    for path in paths:
        candidate = path if path.is_absolute() else Path.cwd() / path
        if not candidate.is_file():
            sys.exit(f"input not found: {candidate}")
        if candidate.suffix != ".mmd":
            sys.exit(f"expected .mmd file: {candidate}")
        resolved.append(candidate.resolve())
    return resolved


def render_one(
    mmdc_argv: list[str],
    input_path: Path,
    output_path: Path,
    fmt: str,
    width: int | None,
    height: int | None,
    background: str,
    dry_run: bool,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        *mmdc_argv,
        "-i",
        str(input_path),
        "-o",
        str(output_path),
        "-b",
        background,
    ]
    if DEFAULT_PUPPETEER_CONFIG.is_file():
        cmd.extend(["-p", str(DEFAULT_PUPPETEER_CONFIG)])
    if width is not None:
        cmd.extend(["-w", str(width)])
    if height is not None:
        cmd.extend(["-H", str(height)])

    print(f"render {input_path.name} -> {output_path}")
    if dry_run:
        print("  ", " ".join(cmd))
        return

    env = mmdc_env()
    if not dry_run and not find_chromium():
        sys.exit(
            "chromium not found. Install the chromium package or set PUPPETEER_EXECUTABLE_PATH."
        )

    try:
        subprocess.run(cmd, check=True, env=env)
    except subprocess.CalledProcessError as exc:
        sys.exit(f"mmdc failed for {input_path} (exit {exc.returncode})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate PDF/PNG/SVG exports from Mermaid .mmd diagram sources."
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help=".mmd files to render (omit when using --all)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help=f"render every top-level .mmd file in {DEFAULT_DIAGRAMS_DIR}",
    )
    parser.add_argument(
        "--embed-algorithms",
        action="store_true",
        help=(
            f"render {ALGORITHMS_DIAGRAMS_DIR}/*.mmd as PNG into "
            f"{EMBEDDED_ALGORITHM_IMAGES_DIR} (committed; used by ALGORITHM.md)"
        ),
    )
    parser.add_argument(
        "--diagrams-dir",
        type=Path,
        default=DEFAULT_DIAGRAMS_DIR,
        help="directory containing .mmd sources",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="directory for rendered outputs",
    )
    parser.add_argument(
        "--format",
        choices=("pdf", "png", "svg"),
        default="pdf",
        help="output format (default: pdf)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help="override canvas width in pixels",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=None,
        help="override canvas height in pixels",
    )
    parser.add_argument(
        "--background",
        default="white",
        help="background color passed to mmdc -b (default: white)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print mmdc commands without executing",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="print mmdc/chromium availability and exit",
    )
    return parser.parse_args()


def check_tools(*, dry_run: bool) -> None:
    try:
        argv = resolve_mmdc_argv(dry_run=dry_run)
    except SystemExit:
        print("mmdc: missing")
        return
    print(f"mmdc:     {' '.join(argv)}")
    chromium = find_chromium()
    print(f"chromium: {chromium or 'missing'}")


def collect_algorithm_inputs() -> list[Path]:
    if not ALGORITHMS_DIAGRAMS_DIR.is_dir():
        sys.exit(f"algorithms diagrams directory not found: {ALGORITHMS_DIAGRAMS_DIR}")
    inputs = sorted(ALGORITHMS_DIAGRAMS_DIR.glob("*.mmd"))
    if not inputs:
        sys.exit(f"no .mmd files in {ALGORITHMS_DIAGRAMS_DIR}")
    return inputs


def main() -> None:
    args = parse_args()
    if args.check:
        check_tools(dry_run=args.dry_run)
        return
    mmdc_argv = resolve_mmdc_argv(dry_run=args.dry_run)

    if args.embed_algorithms:
        inputs = collect_algorithm_inputs()
        output_dir = EMBEDDED_ALGORITHM_IMAGES_DIR
        fmt = "png"
    else:
        inputs = collect_inputs(args.inputs, args.diagrams_dir.resolve(), args.all)
        output_dir = args.output_dir.resolve()
        fmt = args.format

    for input_path in inputs:
        hints = RENDER_HINTS.get(input_path.name, {})
        width = args.width if args.width is not None else hints.get("width")
        height = args.height if args.height is not None else hints.get("height")
        output_path = output_dir / f"{input_path.stem}.{fmt}"
        render_one(
            mmdc_argv,
            input_path,
            output_path,
            fmt,
            width,
            height,
            args.background,
            args.dry_run,
        )

    print(f"done ({len(inputs)} diagram(s))")


if __name__ == "__main__":
    main()
