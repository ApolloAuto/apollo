const { devServerConfig, mainConfig } = require('@dreamview/dreamview-web/config/webpackConfig.js');
const packageJson = require('../package.json');
const { outputPath, appSrcPath, enterPoint } = require('./paths');
/**
 * @typedef {Object} Params
 * @property {string} outputPath - webpack打包输出路径
 * @property {string} contextPath - webpack打包上下文路径
 * @property {string} entrypoint - webpack打包入口文件
 * @property {string} prodSourceMap - webpack打包生产环境sourceMap
 * @property {string} version - webpack打包版本号
 * */

/**
 * @type {Params}
 */
const params = {
    outputPath,
    contextPath: appSrcPath,
    entrypoint: enterPoint,
    prodSourceMap: false,
    version: packageJson.version,
};

module.exports = [devServerConfig(params), mainConfig(params)];
