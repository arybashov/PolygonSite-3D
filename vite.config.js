import { resolve } from 'node:path';
import { defineConfig } from 'vite';

export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        whitepaper: resolve(__dirname, 'whitepaper.html'),
      },
    },
  },
});
