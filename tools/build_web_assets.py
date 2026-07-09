#!/usr/bin/env python3
"""Build web-sized MuJoCo assets for the GARMI sim page.

Reads the `mujoco/` folder of a garmi_description checkout and writes a
slimmed copy to public/garmi-sim/:

  - scene.xml / garmi.xml   copied verbatim (+ mirrored-cover patch below)
  - *.stl                   welded, optionally decimated, binary STL
  - *.obj                   welded, optionally decimated, precision-reduced,
                            angle-split normals; UVs kept where the MJCF
                            binds an image texture
  - *.png textures          downscaled to --tex-size
  - manifest.json           file list the JS loader fetches into the VFS

Meshes are stored pre-gzipped (*.obj.gz / *.stl.gz) because GitHub Pages does
not compress those content types; the loader inflates them with the browser's
DecompressionStream. The manifest lists logical names plus which are gzipped.

--decimate RATIO (default 0.35, 0 disables) quadric-decimates each mesh to
RATIO of its triangles (meshes under --decimate-floor faces are left alone).
Welding happens first: the source OBJs are unwelded "triangle soup", and
decimating soup tears every seam (we learned this the hard way). Decimation
exists to keep MuJoCo's in-browser compile memory down (~1.3 kB of wasm heap
scratch per triangle in MuJoCo 3.x), which is what strands 32-bit mobile
browsers; it also shrinks files and speeds up loading everywhere.

Usage:
    python tools/build_web_assets.py path/to/garmi_description/garmi_description/mujoco
"""
import argparse
import fnmatch
import gzip
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh

try:
    import fast_simplification
except ImportError:
    fast_simplification = None
try:
    import meshoptimizer
except ImportError:
    meshoptimizer = None
from PIL import Image

# The URDF instances the base's side/end covers twice (second one yawed 180°),
# but garmi.xml currently has each only once, leaving the base open at the
# rear and left. Patch the web copy until it's fixed upstream; this is a no-op
# once garmi.xml contains the mirrored instances itself.
MIRRORED_COVER_GEOMS = ["side_cover", "end_cover"]

# OBJs whose UVs must survive (their MJCF materials carry image textures).
KEEP_UV_GLOBS = ["body/*", "head/*"]

V_DECIMALS = 4    # 0.1 mm -- plenty for visual meshes
VT_DECIMALS = 4
VN_DECIMALS = 3
SMOOTH_ANGLE_DEG = 35  # dihedral threshold for smooth vs. crisp normals

FACE_RE = re.compile(r"(-?\d+)(?:/(-?\d*))?(?:/(-?\d+))?")


def parse_obj(src: Path, keep_uv: bool):
    """Parse v/vt/f into arrays; faces reference (vert, uv) per corner."""
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
                corners.append((vi - 1, int(ti) - 1 if (ti and keep_uv) else -1))
            for i in range(1, len(corners) - 1):
                faces.append((corners[0], corners[i], corners[i + 1]))
    v = np.asarray(verts, dtype=np.float64)
    vt = np.asarray(uvs, dtype=np.float64) if uvs else None
    f = np.asarray(faces, dtype=np.int64)  # (n, 3, 2): vert id, uv id
    return v, vt, f


def weld(v, vt, f):
    """Merge corners with identical quantized (position[, uv]).

    Returns welded vertex positions, per-vertex uv (or None) and (n,3) faces.
    Texture seams stay split because uv participates in the key.
    """
    vq = np.round(v, V_DECIMALS)
    corner_v = f[:, :, 0].reshape(-1)
    corner_t = f[:, :, 1].reshape(-1)
    if vt is not None:
        tq = np.round(vt, VT_DECIMALS)
        keys = np.concatenate(
            [vq[corner_v], np.where(corner_t[:, None] >= 0, tq[np.maximum(corner_t, 0)], 0.0)],
            axis=1)
    else:
        keys = vq[corner_v]
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    out_v = uniq[:, :3]
    out_t = uniq[:, 3:5] if vt is not None else None
    out_f = inverse.reshape(-1, 3)
    # drop faces that collapsed under quantization
    good = (out_f[:, 0] != out_f[:, 1]) & (out_f[:, 1] != out_f[:, 2]) & (out_f[:, 0] != out_f[:, 2])
    return out_v, out_t, out_f[good].astype(np.int32)


def decimate(v, t, f, ratio, floor):
    """Quadric decimation on a welded mesh. UVs survive via collapse replay."""
    n = len(f)
    target = int(max(floor, n * ratio))
    if fast_simplification is None or ratio <= 0 or n <= target:
        return v, t, f
    if t is None:
        out_v, out_f = fast_simplification.simplify(
            v.astype(np.float32), f, target_count=target)
        return out_v.astype(np.float64), None, out_f.astype(np.int32)
    # Textured: meshoptimizer with locked borders. Boundary and uv-seam
    # vertices cannot move or collapse, so seams stay closed (our earlier
    # collapse-replay approach opened holes along them), and the surviving
    # indices point into the original vertex buffer, so uvs carry over 1:1.
    if meshoptimizer is None:
        sys.exit("error: textured decimation needs `pip install meshoptimizer`")
    dest = np.zeros(len(f) * 3, dtype=np.uint32)
    count = meshoptimizer.simplify(
        dest, f.astype(np.uint32).ravel(), v.astype(np.float32),
        target_index_count=target * 3, target_error=0.02,
        options=meshoptimizer.SIMPLIFY_LOCK_BORDER)
    nf = dest[:count].reshape(-1, 3).astype(np.int64)
    used, inv = np.unique(nf, return_inverse=True)
    return v[used], t[used], inv.reshape(-1, 3).astype(np.int32)


def corner_normals(v, f, angle_deg=SMOOTH_ANGLE_DEG):
    """Per-corner normals: average adjacent face normals within the angle
    threshold of the corner's own face -- smooth on curvature, crisp on edges.
    Returns (unique_normals, (n,3) normal index per corner)."""
    e1 = v[f[:, 1]] - v[f[:, 0]]
    e2 = v[f[:, 2]] - v[f[:, 0]]
    fn = np.cross(e1, e2)
    norm = np.linalg.norm(fn, axis=1, keepdims=True)
    fn = fn / np.maximum(norm, 1e-30)
    vert_faces = defaultdict(list)
    for fi, tri in enumerate(f):
        for vi in tri:
            vert_faces[int(vi)].append(fi)
    cos_t = math.cos(math.radians(angle_deg))
    uniq, nidx = {}, np.zeros_like(f)
    for fi, tri in enumerate(f):
        for c, vi in enumerate(tri):
            adj = vert_faces[int(vi)]
            dots = fn[adj] @ fn[fi]
            n = fn[adj][dots > cos_t].sum(axis=0)
            n /= max(np.linalg.norm(n), 1e-30)
            key = tuple(np.round(n, VN_DECIMALS))
            nidx[fi, c] = uniq.setdefault(key, len(uniq))
    return np.array(list(uniq.keys())), nidx


def export_obj(dst: Path, v, t, f, src_name: str):
    normals, nidx = corner_normals(v, f)
    with dst.open("w") as out:
        out.write(f"# web-slimmed from {src_name}\n")
        for x, y, z in v:
            out.write(f"v {x:.{V_DECIMALS}f} {y:.{V_DECIMALS}f} {z:.{V_DECIMALS}f}\n")
        if t is not None:
            for u, w in t:
                out.write(f"vt {u:.{VT_DECIMALS}f} {w:.{VT_DECIMALS}f}\n")
        for x, y, z in normals:
            out.write(f"vn {x:.{VN_DECIMALS}f} {y:.{VN_DECIMALS}f} {z:.{VN_DECIMALS}f}\n")
        for fi, tri in enumerate(f):
            toks = []
            for c, vi in enumerate(tri):
                if t is not None:
                    toks.append(f"{vi + 1}/{vi + 1}/{nidx[fi, c] + 1}")
                else:
                    toks.append(f"{vi + 1}//{nidx[fi, c] + 1}")
            out.write("f " + " ".join(toks) + "\n")


def process_obj(src: Path, dst: Path, keep_uv: bool, ratio: float,
                ratio_textured: float, floor: int):
    v, vt, fc = parse_obj(src, keep_uv)
    v, t, f = weld(v, vt, fc)
    n_in = len(f)
    v, t, f = decimate(v, t, f, ratio_textured if keep_uv else ratio, floor)
    export_obj(dst, v, t, f, src.name)
    return n_in, len(f)


def process_stl(src: Path, dst: Path, ratio: float, floor: int):
    mesh = trimesh.load(src, force="mesh", process=False)
    v, _, f = weld(mesh.vertices, None,
                   np.stack([mesh.faces, np.full_like(mesh.faces, -1)], axis=2))
    n_in = len(f)
    v, _, f = decimate(v, None, f, ratio, floor)
    trimesh.Trimesh(vertices=v, faces=f, process=False).export(dst)
    return n_in, len(f)


def patch_mirrored_covers(xml: str) -> str:
    """Add 180°-yawed duplicates of single-instance cover geoms (see above)."""
    for mesh in MIRRORED_COVER_GEOMS:
        geoms = re.findall(rf'<geom mesh="{mesh}"[^/]*/>', xml)
        if len(geoms) == 1 and 'quat' not in geoms[0]:
            mirrored = geoms[0][:-2] + ' quat="0 0 0 1"/>'
            xml = xml.replace(geoms[0], geoms[0] + "\n      " + mirrored)
            print(f"  patched   mirrored {mesh} geom added (missing upstream)")
    return xml


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
    ap.add_argument("--decimate", type=float, default=0.3, metavar="RATIO",
                    help="triangle ratio to keep per mesh (0 disables)")
    ap.add_argument("--decimate-textured", type=float, default=0.0, metavar="RATIO",
                    help="ratio for uv-textured meshes (0 = keep full detail; "
                         "collapse-based uv carry creases fabric below ~0.5)")
    ap.add_argument("--decimate-floor", type=int, default=3500,
                    help="meshes at or below this many faces are not decimated")
    args = ap.parse_args()

    if args.decimate > 0 and fast_simplification is None:
        sys.exit("error: --decimate needs `pip install fast-simplification`")

    src_root = args.mujoco_dir.resolve()
    assets = src_root / "assets"
    if not (src_root / "scene.xml").exists() or not assets.is_dir():
        sys.exit(f"error: {src_root} does not look like the garmi mujoco folder")

    out_root = args.out.resolve()
    (out_root / "assets").mkdir(parents=True, exist_ok=True)

    manifest, gzipped, total_in, total_out = [], [], 0, 0
    tris_in = tris_out = 0

    def emit(rel: str, src: Path, dst: Path):
        nonlocal total_in, total_out
        manifest.append(rel)
        total_in += src.stat().st_size
        total_out += dst.stat().st_size

    def emit_gz(rel: str, src: Path, dst: Path):
        data = dst.read_bytes()
        gz = dst.parent / (dst.name + ".gz")
        gz.write_bytes(gzip.compress(data, 9, mtime=0))
        dst.unlink()
        gzipped.append(rel)
        emit(rel, src, gz)

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
        rel = src.relative_to(src_root)
        dst = out_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        suffix = src.suffix.lower()
        rel_in_assets = str(src.relative_to(assets))

        if suffix in (".obj", ".stl"):
            if suffix == ".obj":
                keep_uv = any(fnmatch.fnmatch(rel_in_assets, g) for g in KEEP_UV_GLOBS)
                n_in, n_out = process_obj(src, dst, keep_uv, args.decimate,
                                          args.decimate_textured, args.decimate_floor)
            else:
                n_in, n_out = process_stl(src, dst, args.decimate, args.decimate_floor)
            tris_in += n_in
            tris_out += n_out
            emit_gz(str(rel), src, dst)
            print(f"  {suffix[1:]:4} {rel_in_assets:34} {n_in:>6} -> {n_out:>6} tris  "
                  f"{src.stat().st_size/1e6:6.2f} -> {(out_root / (str(rel) + '.gz')).stat().st_size/1e6:5.2f} MB gz")
        elif suffix in (".png", ".jpg", ".jpeg"):
            convert_png(src, dst, args.tex_size)
            emit(str(rel), src, dst)
            print(f"  tex  {rel_in_assets:34} "
                  f"{src.stat().st_size/1e6:6.2f} -> {dst.stat().st_size/1e6:5.2f} MB")
        else:
            dst.write_bytes(src.read_bytes())
            emit(str(rel), src, dst)

    (out_root / "manifest.json").write_text(
        json.dumps({"scene": "scene.xml", "files": manifest, "gzipped": gzipped},
                   indent=1))

    print(f"\ntriangles: {tris_in} -> {tris_out} (welded input; ratio {args.decimate})")
    print(f"total: {total_in/1e6:.1f} MB -> {total_out/1e6:.1f} MB "
          f"({len(manifest)} files) -> {out_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
