import { defineConfig } from 'vitest/config';
import { fileURLToPath } from 'node:url';
const path = (value:string) => fileURLToPath(new URL(value,import.meta.url));
export default defineConfig({
  esbuild: {jsx:'automatic'},
  resolve: {mainFields:['module','main'],dedupe:['react','react-dom'],alias: {
    '@':path('../upstream/phosphobot/dashboard/src'),
    'three':path('../upstream/phosphobot/dashboard/node_modules/three'),
    'react':path('./node_modules/react'),
    'react-dom':path('./node_modules/react-dom'),
    'swr':path('./node_modules/swr'),
    'lucide-react':path('../upstream/phosphobot/dashboard/node_modules/lucide-react/dist/esm/lucide-react.js'),
    'react-router/dom':path('../upstream/phosphobot/dashboard/node_modules/react-router/dist/development/dom-export.mjs'),
    'react-router':path('../upstream/phosphobot/dashboard/node_modules/react-router/dist/development/index.mjs'),
    'react-router-dom':path('../upstream/phosphobot/dashboard/node_modules/react-router-dom/dist/index.mjs'),
    'jsqr':path('../upstream/phosphobot/dashboard/node_modules/jsqr/dist/jsQR.js')
  }},
  // Upstream Radix is installed beside upstream React. Inline its dependencies
  // so the fixture's alias selects the same React instance as its renderer.
  test: {environment:'jsdom',setupFiles:['./setup.ts'],include:['*.test.tsx'],restoreMocks:true,
    server:{deps:{inline:[/react-router/,/@floating-ui/,/@radix-ui/,/react-remove-scroll/,/react-style-singleton/,/use-sidecar/,/use-callback-ref/]}}}
});
