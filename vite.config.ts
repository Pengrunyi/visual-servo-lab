import { defineConfig } from "vitest/config";

export default defineConfig({
  // Required so the production build works on GitHub Pages at
  // https://pengrunyi.github.io/visual-servo-lab/
  base: "/visual-servo-lab/",
  server: {
    port: 5173,
    host: true,
  },
  build: {
    // Three.js is the bulk of the bundle; 500 kB is expected for the 3D view.
    chunkSizeWarningLimit: 600,
  },
  test: {
    include: ["src/**/*.test.ts"],
  },
});
