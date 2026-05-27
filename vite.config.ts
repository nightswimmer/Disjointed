import { defineConfig } from "vite";

export default defineConfig({
  // Use a relative base so the built site works when served from any subpath.
  base: "./",
  server: {
    open: true,
  },
});
