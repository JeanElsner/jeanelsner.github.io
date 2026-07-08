# jeanelsner.github.io

Personal research website with an interactive, browser-based MuJoCo simulation
of the [GARMI](https://github.com/geriatronics/garmi_description) assistive
humanoid. Static site, no backend — plain HTML/CSS + [Vite](https://vite.dev),
deployed to GitHub Pages by CI.

## Pages

- `index.html` — landing page.
- `garmi/index.html` + `src/garmi/` — the simulation. The official
  [MuJoCo WASM bindings](https://github.com/google-deepmind/mujoco/tree/main/wasm)
  (`@mujoco/mujoco`, single-threaded build) step the full GARMI model
  (94 joints, physical mecanum rollers) at 500 Hz; three.js renders it.
  - `sim.js` — asset loading into the MuJoCo VFS, main loop, UI wiring.
  - `mjRender.js` — mirrors mjModel geoms/materials/textures into three.js;
    per-frame it only writes transforms from the live `geom_xpos/geom_xmat` views.
  - `teleop.js` — port of `garmi_description/mujoco/teleop.py`: mecanum IK +
    feedforward/proportional twist controller on the measured base twist.
  - In the browser console, `window.__garmi` exposes `{ mujoco, model, data }`.

## Develop

```sh
npm install
npm run dev        # http://localhost:5173
npm run build      # production build in dist/
npm run preview    # serve dist/ locally
```

## Robot model assets

`public/garmi-sim/` holds a web-slimmed copy of the model (~6 MB, committed
pre-gzipped: GitHub Pages doesn't compress `.obj`/`.stl`, so meshes are stored
as `.gz` and inflated in the browser via `DecompressionStream`). Regenerate
after upstream model changes:

```sh
pip install trimesh pillow
git clone https://github.com/geriatronics/garmi_description /tmp/garmi_description
python tools/build_web_assets.py /tmp/garmi_description/garmi_description/mujoco
```

The pipeline reduces OBJ float precision (dropping normals, and UVs where no
texture uses them), downsizes textures and writes `manifest.json`, which the
sim fetches. It deliberately does **not** decimate: the visual OBJs are
unwelded triangle soup, so quadric decimation tears them open at the seams
(the base cover lost its back panel when we tried).

It also patches `garmi.xml`: upstream currently instances the base's
side/end covers once, while the URDF mirrors each with a 180° yaw — without
the patch the base is open at the rear and left. The patch is idempotent and
becomes a no-op once garmi_description adds the mirrored geoms.

## Deploy

Pushed to `main` → `.github/workflows/deploy.yml` builds and publishes via
GitHub Pages (repo Settings → Pages → Source: **GitHub Actions**).

Notes:
- For a user site (`jeanelsner.github.io`) the Vite `base` stays `/`. If this
  ever moves to a project repo, set `base: '/<repo>/'` in `vite.config.js`.
- GitHub Pages cannot send COOP/COEP headers, so the multi-threaded MuJoCo
  build (`@mujoco/mujoco/mt`, needs `SharedArrayBuffer`) is not an option
  here — the single-threaded build runs this model at >20× realtime anyway.

## Licensing

Site code: MIT. The GARMI model and meshes are © Technical University of
Munich, Apache-2.0, with FR3/Franka Hand assets from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
(Apache-2.0) — see the `garmi_description` repository's NOTICE files.
