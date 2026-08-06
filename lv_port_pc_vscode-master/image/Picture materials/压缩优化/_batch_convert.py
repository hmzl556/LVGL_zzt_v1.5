# -*- coding: utf-8 -*-
"""Batch convert picture/ assets into 压缩优化/ with category rules."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(r"F:/A_ZZT_Project/Xiaoya_Xiyiji/LVGL_zzt_v1.5/lv_port_pc_vscode-master")
SCRIPT = ROOT / "lvgl" / "scripts" / "LVGLImage.py"
SRC_ROOT = ROOT / "image" / "Picture materials" / "picture"
OUT_ROOT = ROOT / "image" / "Picture materials" / "压缩优化"

IMG_EXT = {".png", ".jpg", ".jpeg", ".bmp"}

JOBS = [
    # (subdir under picture/, cf, compress, note)
    ("Monochrome_or_Few_Colors", "RGB565A8", "LZ4", "few-color icons + alpha"),
    ("multicolour", "RGB565A8", "LZ4", "multicolour + alpha"),
    ("program", "RGB565A8", "LZ4", "program images"),
    ("qr-codes", "RGB565", "LZ4", "QR png only"),
]


def convert_one(png: Path, out_dir: Path, ofmt: str, cf: str, compress: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(SCRIPT),
        "--ofmt",
        ofmt,
        "--cf",
        cf,
        "--compress",
        compress,
        "-o",
        str(out_dir),
        "--name",
        png.stem,
        str(png),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(
            f"FAIL {png}\ncmd={' '.join(cmd)}\nstdout={r.stdout}\nstderr={r.stderr}"
        )


def main() -> None:
    if not SCRIPT.is_file():
        raise SystemExit(f"missing script: {SCRIPT}")
    if not SRC_ROOT.is_dir():
        raise SystemExit(f"missing src: {SRC_ROOT}")

    # I1/I2/I4/I8 need pngquant.exe on PATH (Windows)
    pngquant_dir = ROOT / "tools" / "pngquant" / "pngquant"
    if (pngquant_dir / "pngquant.exe").is_file():
        import os

        os.environ["PATH"] = str(pngquant_dir) + os.pathsep + os.environ.get("PATH", "")

    only = set(sys.argv[1:])  # optional: pass category folder names to limit
    jobs = JOBS
    if only:
        jobs = [j for j in JOBS if j[0] in only]
        if not jobs:
            raise SystemExit(f"no matching jobs for {only}")

    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    summary: list[str] = []
    ok = 0
    fail = 0

    for sub, cf, compress, note in jobs:
        src_dir = SRC_ROOT / sub
        if not src_dir.is_dir():
            print(f"SKIP missing {src_dir}")
            continue

        images = sorted(
            p
            for p in src_dir.rglob("*")
            if p.is_file() and p.suffix.lower() in IMG_EXT
        )
        # qr-codes: png only (already filtered); user asked png specifically
        if sub == "qr-codes":
            images = [p for p in images if p.suffix.lower() == ".png"]

        print(f"\n=== {sub}: {len(images)} files | --cf {cf} --compress {compress} ({note}) ===")
        summary.append(f"## {sub}")
        summary.append(f"- format: cf={cf}, compress={compress}")
        summary.append(f"- count: {len(images)}")
        summary.append("")

        for png in images:
            rel = png.relative_to(src_dir)
            # preserve nested relative folders under c/ and bin/
            out_c = OUT_ROOT / sub / "c" / rel.parent
            out_bin = OUT_ROOT / sub / "bin" / rel.parent
            try:
                convert_one(png, out_c, "C", cf, compress)
                convert_one(png, out_bin, "BIN", cf, compress)
                c_file = out_c / f"{png.stem}.c"
                b_file = out_bin / f"{png.stem}.bin"
                c_sz = c_file.stat().st_size if c_file.is_file() else 0
                b_sz = b_file.stat().st_size if b_file.is_file() else 0
                line = f"OK {rel.as_posix()}  c={c_sz/1024:.1f}KB  bin={b_sz/1024:.1f}KB"
                print(line)
                summary.append(f"- {rel.as_posix()}: c={c_sz/1024:.1f}KB, bin={b_sz/1024:.1f}KB")
                ok += 1
            except Exception as e:
                fail += 1
                print(f"ERR {rel}: {e}")
                summary.append(f"- ERROR {rel.as_posix()}: {e}")

        summary.append("")

    summary_path = OUT_ROOT / "batch_convert_summary.txt"
    header = [
        "Batch convert summary",
        f"src: {SRC_ROOT}",
        f"out: {OUT_ROOT}",
        f"ok={ok} fail={fail}",
        "",
        "Rules:",
        "- Monochrome_or_Few_Colors: RGB565A8 + LZ4",
        "- multicolour: RGB565A8 + LZ4",
        "- program: RGB565A8 + LZ4",
        "- qr-codes: RGB565 + LZ4 (png only)",
        "",
        "Each category has c/ and bin/ subfolders.",
        "",
    ]
    summary_path.write_text("\n".join(header + summary), encoding="utf-8")
    print(f"\nDONE ok={ok} fail={fail}")
    print(f"summary: {summary_path}")


if __name__ == "__main__":
    main()
