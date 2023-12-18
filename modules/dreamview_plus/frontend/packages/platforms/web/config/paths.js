const path = require('path');

const rootPath = path.join(__dirname, '../');

const outputPath = path.join(__dirname, '../.web_build');

const frontendPath = path.join(__dirname, '../../../../../frontend');

// 源码位置
const appSrcPath = path.join(__dirname, '../lib');
const enterPoint = path.join(appSrcPath, './web.tsx');

module.exports = {
    rootPath,
    outputPath,
    appSrcPath,
    enterPoint,
    frontendPath,
};
