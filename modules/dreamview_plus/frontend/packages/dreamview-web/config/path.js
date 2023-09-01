const path = require('path');

const manifestPath = path.resolve(__dirname, '../public/manifest.json');

const htmlPath = path.resolve(__dirname, '../public/index.html');

const webPublicPath = path.resolve(__dirname, '../public');

module.exports = {
    manifestPath,
    htmlPath,
    webPublicPath,
};
