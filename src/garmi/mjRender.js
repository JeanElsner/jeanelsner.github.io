// Mirrors a compiled MuJoCo model into a three.js scene.
//
// Geometry is built once from mjModel (meshes are expanded to non-indexed
// buffers so per-face normal/uv indices work), then per-frame updates only
// write geom transforms from the live mjData views -- no per-frame allocation
// or embind traffic.
import * as THREE from 'three';

const VISIBLE_GROUP_MAX = 2; // group 3 = collision geoms (rollers, hulls)
const FLOOR_RADIUS = 12;

export class MjThreeScene {
  constructor(mujoco, model, data, scene) {
    this.model = model;
    this.scene = scene;
    this.entries = []; // { object3d, geomId }
    this.geomXpos = data.geom_xpos; // live views into the wasm heap
    this.geomXmat = data.geom_xmat;

    const geomCache = new Map();
    const matCache = new Map();
    const T = mujoco.mjtGeom;

    for (let g = 0; g < model.ngeom; g++) {
      if (model.geom_group[g] > VISIBLE_GROUP_MAX) continue;
      const rgba = this.#geomRgba(g);
      if (rgba[3] === 0) continue;
      const type = model.geom_type[g];

      if (type === T.mjGEOM_PLANE.value) {
        const floor = makeFloor();
        this.scene.add(floor);
        this.entries.push({ object3d: floor, geomId: g });
        continue;
      }

      let geometry = null;
      if (type === T.mjGEOM_MESH.value) {
        const meshId = model.geom_dataid[g];
        geometry = geomCache.get(`m${meshId}`) ?? this.#buildMeshGeometry(meshId);
        geomCache.set(`m${meshId}`, geometry);
      } else {
        const s = model.geom_size.subarray(3 * g, 3 * g + 3);
        const key = `p${type}:${s[0]},${s[1]},${s[2]}`;
        geometry = geomCache.get(key) ?? buildPrimitive(T, type, s);
        if (!geometry) continue;
        geomCache.set(key, geometry);
      }

      const matId = model.geom_matid[g];
      const matKey = matId >= 0 ? `mat${matId}` : `rgba${rgba.join()}`;
      let material = matCache.get(matKey);
      if (!material) {
        material = this.#buildMaterial(mujoco, matId, rgba);
        matCache.set(matKey, material);
      }

      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.matrixAutoUpdate = false;
      this.scene.add(mesh);
      this.entries.push({ object3d: mesh, geomId: g });
    }

    this.update();
  }

  #geomRgba(g) {
    const m = this.model;
    const matId = m.geom_matid[g];
    const src = matId >= 0 ? m.mat_rgba.subarray(4 * matId, 4 * matId + 4)
                           : m.geom_rgba.subarray(4 * g, 4 * g + 4);
    return [src[0], src[1], src[2], src[3]];
  }

  // Non-indexed expansion: MuJoCo keeps separate per-face indices for
  // vertices, normals and texcoords, which three.js indexed buffers can't
  // express.
  #buildMeshGeometry(meshId) {
    const m = this.model;
    const vAdr = m.mesh_vertadr[meshId];
    const fAdr = m.mesh_faceadr[meshId];
    const fNum = m.mesh_facenum[meshId];
    const nAdr = m.mesh_normaladr[meshId];
    const tAdr = m.mesh_texcoordadr[meshId];

    const positions = new Float32Array(fNum * 9);
    const normals = new Float32Array(fNum * 9);
    const uvs = tAdr >= 0 ? new Float32Array(fNum * 6) : null;

    for (let f = 0; f < fNum; f++) {
      for (let c = 0; c < 3; c++) {
        const k = 3 * (fAdr + f) + c;
        const out = 9 * f + 3 * c;
        const vi = 3 * (vAdr + m.mesh_face[k]);
        positions[out] = m.mesh_vert[vi];
        positions[out + 1] = m.mesh_vert[vi + 1];
        positions[out + 2] = m.mesh_vert[vi + 2];
        const ni = 3 * (nAdr + m.mesh_facenormal[k]);
        normals[out] = m.mesh_normal[ni];
        normals[out + 1] = m.mesh_normal[ni + 1];
        normals[out + 2] = m.mesh_normal[ni + 2];
        if (uvs) {
          const ti = 2 * (tAdr + m.mesh_facetexcoord[k]);
          uvs[6 * f + 2 * c] = m.mesh_texcoord[ti];
          uvs[6 * f + 2 * c + 1] = m.mesh_texcoord[ti + 1];
        }
      }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    if (uvs) geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    return geo;
  }

  #buildMaterial(mujoco, matId, rgba) {
    const m = this.model;
    const params = {
      color: new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace),
      transparent: rgba[3] < 1,
      opacity: rgba[3],
      roughness: 0.7,
      metalness: 0.05,
      side: THREE.DoubleSide, // decimated shells can have locally flipped tris
    };
    if (matId >= 0) {
      params.roughness = THREE.MathUtils.clamp(1 - 0.85 * m.mat_shininess[matId], 0.15, 1);
      params.metalness = THREE.MathUtils.clamp(m.mat_reflectance[matId], 0, 0.4);
      const nRole = mujoco.mjtTextureRole.mjNTEXROLE.value;
      const rgbRole = mujoco.mjtTextureRole.mjTEXROLE_RGB.value;
      const texId = m.mat_texid[matId * nRole + rgbRole];
      if (texId >= 0) {
        params.map = this.#buildTexture(texId);
        params.color = new THREE.Color(1, 1, 1);
      }
    }
    return new THREE.MeshStandardMaterial(params);
  }

  #buildTexture(texId) {
    const m = this.model;
    // Some address/size arrays are mjtSize-backed (BigInt64) in the bindings.
    const w = Number(m.tex_width[texId]);
    const h = Number(m.tex_height[texId]);
    const nc = Number(m.tex_nchannel[texId]);
    const adr = Number(m.tex_adr[texId]);
    const src = m.tex_data;
    const rgba = new Uint8Array(w * h * 4);
    for (let i = 0; i < w * h; i++) {
      rgba[4 * i] = src[adr + nc * i];
      rgba[4 * i + 1] = src[adr + nc * i + 1];
      rgba[4 * i + 2] = src[adr + nc * i + 2];
      rgba[4 * i + 3] = nc === 4 ? src[adr + nc * i + 3] : 255;
    }
    const tex = new THREE.DataTexture(rgba, w, h, THREE.RGBAFormat);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.magFilter = THREE.LinearFilter;
    tex.minFilter = THREE.LinearMipmapLinearFilter;
    tex.generateMipmaps = true;
    tex.anisotropy = 4;
    tex.needsUpdate = true;
    return tex;
  }

  update() {
    const xpos = this.geomXpos;
    const xmat = this.geomXmat;
    for (const { object3d, geomId: g } of this.entries) {
      object3d.matrix.set(
        xmat[9 * g], xmat[9 * g + 1], xmat[9 * g + 2], xpos[3 * g],
        xmat[9 * g + 3], xmat[9 * g + 4], xmat[9 * g + 5], xpos[3 * g + 1],
        xmat[9 * g + 6], xmat[9 * g + 7], xmat[9 * g + 8], xpos[3 * g + 2],
        0, 0, 0, 1,
      );
      object3d.matrixWorldNeedsUpdate = true;
    }
  }
}

function buildPrimitive(T, type, s) {
  let geo = null;
  if (type === T.mjGEOM_SPHERE.value) {
    geo = new THREE.SphereGeometry(s[0], 24, 16);
  } else if (type === T.mjGEOM_CAPSULE.value) {
    geo = new THREE.CapsuleGeometry(s[0], 2 * s[1], 8, 16);
    geo.rotateX(Math.PI / 2);
  } else if (type === T.mjGEOM_CYLINDER.value) {
    geo = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24);
    geo.rotateX(Math.PI / 2);
  } else if (type === T.mjGEOM_BOX.value) {
    geo = new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
  } else if (type === T.mjGEOM_ELLIPSOID.value) {
    geo = new THREE.SphereGeometry(1, 24, 16);
    geo.scale(s[0], s[1], s[2]);
  }
  return geo;
}

// A soft checkered disc that fades out at the edge (nicer on a page than
// MuJoCo's infinite plane; the physics floor underneath is still infinite).
function makeFloor() {
  const size = 1024;
  const squares = 32;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#c8cdd4';
  ctx.fillRect(0, 0, size, size);
  ctx.fillStyle = '#b2bac5';
  const cell = size / squares;
  for (let i = 0; i < squares; i++) {
    for (let j = 0; j < squares; j++) {
      if ((i + j) % 2) ctx.fillRect(i * cell, j * cell, cell, cell);
    }
  }
  const fade = ctx.createRadialGradient(
    size / 2, size / 2, size * 0.25, size / 2, size / 2, size * 0.5);
  fade.addColorStop(0, 'rgba(255,255,255,1)');
  fade.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.globalCompositeOperation = 'destination-in';
  ctx.fillStyle = fade;
  ctx.fillRect(0, 0, size, size);

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  const mesh = new THREE.Mesh(
    new THREE.CircleGeometry(FLOOR_RADIUS, 64),
    new THREE.MeshStandardMaterial({
      map: tex, transparent: true, roughness: 0.95, metalness: 0,
    }),
  );
  mesh.receiveShadow = true;
  mesh.matrixAutoUpdate = false;
  return mesh;
}
