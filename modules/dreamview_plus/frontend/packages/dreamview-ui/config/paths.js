const path = require('path');

const appRoot = path.join(__dirname, '../');
const resolveApp = (relativePath) => path.resolve(appRoot, relativePath);

module.exports = {
    alias: {
        '@': resolveApp('src'),
        components: resolveApp('./src/components'),
        src: resolveApp('src'),
    },
};
