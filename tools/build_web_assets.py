#!/usr/bin/env python3
"""Build web-sized MuJoCo assets for the GARMI sim page.

Reads the `mujoco/` folder of a garmi_description checkout and writes a
slimmed copy to public/garmi-sim/:

  - scene.xml / garmi.xml   copied verbatim
  - *.stl                   re-exported as binary STL (geometry untouched)
  - *.obj                   text rewrite: float precision reduced, normals
                            dropped (MuJoCo regenerates them), UVs kept only
                            where the MJCF binds an image texture
  - *.png textures          downscaled to --tex-size
  - manifest.json           file list the JS loader fetches into the VFS

No mesh decimation. The visual OBJs (obj2mjcf / CAD exports) are unwelded
"triangle soup" -- coincident edges are exact duplicates, not shared vertices
-- so quadric decimation tears them open at every seam (we tried; the base
cover lost its back panel). Precision-reduced soup renders identically to the
original, and gzip on the wire makes the remaining size acceptable.

Usage:
    python tools/build_web_assets.py path/to/garmi_description/garmi_description/mujoco
"""
import argparse
import fnmatch
import json
import re
import sys
from pathlib import Path

# The URDF instances the base's side/end covers twice (second one yawed 180°),
# but garmi.xml currently has each only once, leaving the base open at the
# rear and left. Patch the web copy until it's fixed upstream; this is a no-op
# once garmi.xml contains the mirrored instances itself.
MIRRORED_COVER_GEOMS = ["side_cover", "end_cover"]

import trimesh
from PIL import Image

# OBJs whose UVs must survive (their MJCF materials carry image textures).
KEEP_UV_GLOBS = ["body/*", "head/*"]

V_DECIMALS = 4   # 0.1 mm -- plenty for visual meshes
VT_DECIMALS = 4

FACE_RE = re.compile(r"(-?\d+)(?:/(-?\d*))?(?:/(-?\d+))?")


def rewrite_obj(src: Path, dst: Path, keep_uv: bool) -> int:
    """Rewrite an OBJ with reduced precision; keep v/f (+vt if keep_uv)."""
    verts, uvs, faces = [], [], []
    for line in src.read_text().splitlines():
        if line.startswith("v "):
            x, y, z = line.split()[1:4]
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("vt ") and keep_uv:
            u, v = line.split()[1:3]
            uvs.append((float(u), float(v)))
        elif line.startswith("f "):
            corners = []
            for tok in line.split()[1:]:
                m = FACE_RE.match(tok)
                vi = int(m.group(1))
                ti = m.group(2)
                corners.append((vi, int(ti) if (ti and keep_uv) else None))
            # obj2mjcf output is triangulated; fan-triangulate defensively.
            for i in range(1, len(corners) - 1):
                faces.append((corners[0], corners[i], corners[i + 1]))

    with dst.open("w") as f:
        f.write(f"# web-slimmed from {src.name}\n")
        for x, y, z in verts:
            f.write(f"v {x:.{V_DECIMALS}f} {y:.{V_DECIMALS}f} {z:.{V_DECIMALS}f}\n")
        for u, v in uvs:
            f.write(f"vt {u:.{VT_DECIMALS}f} {v:.{VT_DECIMALS}f}\n")
        for tri in faces:
            f.write("f " + " ".join(
                f"{vi}/{ti}" if ti else f"{vi}" for vi, ti in tri) + "\n")
    return len(faces)


def patch_mirrored_covers(xml: str) -> str:
    """Add 180°-yawed duplicates of single-instance cover geoms (see above)."""
    for mesh in MIRRORED_COVER_GEOMS:
        geoms = re.findall(rf'<geom mesh="{mesh}"[^/]*/>', xml)
        if len(geoms) == 1 and 'quat' not in geoms[0]:
            mirrored = geoms[0][:-2] + ' quat="0 0 0 1"/>'
            xml = xml.replace(geoms[0], geoms[0] + "\n      " + mirrored)
            print(f"  patched   mirrored {mesh} geom added (missing upstream)")
    return xml


def convert_stl(src: Path, dst: Path) -> None:
    # process=False: pure binary re-encode, geometry bit-identical.
    trimesh.load(src, force="mesh", process=False).export(dst)


def convert_png(src: Path, dst: Path, tex_size: int) -> None:
    img = Image.open(src)
    if max(img.size) > tex_size:
        img.thumbnail((tex_size, tex_size), Image.LANCZOS)
    img.save(dst, optimize=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("mujoco_dir", type=Path,
                    help="path to garmi_description/garmi_description/mujoco")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).resolve().parent.parent / "public/garmi-sim")
    ap.add_argument("--tex-size", type=int, default=1024)
    args = ap.parse_args()

    src_root = args.mujoco_dir.resolve()
    assets = src_root / "assets"
    if not (src_root / "scene.xml").exists() or not assets.is_dir():
        sys.exit(f"error: {src_root} does not look like the garmi mujoco folder")

    out_root = args.out.resolve()
    (out_root / "assets").mkdir(parents=True, exist_ok=True)

    manifest, total_in, total_out = [], 0, 0

    def emit(rel: str, src: Path, dst: Path):
        nonlocal total_in, total_out
        manifest.append(rel)
        total_in += src.stat().st_size
        total_out += dst.stat().st_size

    for name in ("scene.xml", "garmi.xml"):
        dst = out_root / name
        xml = (src_root / name).read_text()
        if name == "garmi.xml":
            xml = patch_mirrored_covers(xml)
        dst.write_text(xml)
        emit(name, src_root / name, dst)

    for src in sorted(assets.rglob("*")):
        if not src.is_file():
            continue
        rel = src.relative_to(src_root)          # e.g. assets/body/body_0.obj
        dst = out_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        suffix = src.suffix.lower()
        rel_in_assets = str(src.relative_to(assets))

        if suffix == ".obj":
            keep_uv = any(fnmatch.fnmatch(rel_in_assets, g) for g in KEEP_UV_GLOBS)
            tris = rewrite_obj(src, dst, keep_uv)
            print(f"  obj{'+uv' if keep_uv else '   '} {rel_in_assets:32} {tris:>7} tris  "
                  f"{src.stat().st_size/1e6:6.2f} -> {dst.stat().st_size/1e6:5.2f} MB")
        elif suffix == ".stl":
            convert_stl(src, dst)
        elif suffix in (".png", ".jpg", ".jpeg"):
            convert_png(src, dst, args.tex_size)
            print(f"  texture {rel_in_assets:34} "
                  f"{src.stat().st_size/1e6:6.2f} -> {dst.stat().st_size/1e6:5.2f} MB")
        else:
            dst.write_bytes(src.read_bytes())
        emit(str(rel), src, dst)

    (out_root / "manifest.json").write_text(
        json.dumps({"scene": "scene.xml", "files": manifest}, indent=1))

    print(f"\ntotal: {total_in/1e6:.1f} MB -> {total_out/1e6:.1f} MB "
          f"({len(manifest)} files) -> {out_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
