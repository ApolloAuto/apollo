const path = require('path');

const appRoot = path.join(__dirname, '../');
const resolveApp = (relativePath) => path.resolve(appRoot, relativePath);
const entryRoot = resolveApp('./src/index.ts');
const outDir = resolveApp('./dist/');

module.exports = {
    alias: {
        '@': resolveApp('src'),
        components: resolveApp('./src/components'),
        src: resolveApp('src'),
    },
    entryRoot,
    outDir,
};
