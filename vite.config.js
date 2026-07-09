import { defineConfig } from 'vite'
import { resolve } from 'node:path'

export default defineConfig({
  // Expose the dev server on the LAN so phones on the same wifi can reach it
  // (Vite binds to 127.0.0.1 by default).
  server: { host: true },
  preview: { host: true },
  // Deployed as a user site (jeanelsner.github.io) → served from the domain
  // root. If you ever deploy this from a project repo instead, set base to
  // '/<repo-name>/'.
  base: '/',
  // The MuJoCo package is an Emscripten module that resolves its .wasm file
  // relative to import.meta.url — esbuild pre-bundling breaks that.
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },
  build: {
    target: 'es2022',
    rollupOptions: {
      input: {
        main: resolve(import.meta.dirname, 'index.html'),
        garmi: resolve(import.meta.dirname, 'garmi/index.html'),
      },
    },
  },
})
