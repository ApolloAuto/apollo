const path = require('path');

const makeConfig = require('@dreamview/dreamview-core/config/webpack.section.js');
const { DefinePlugin } = require('webpack');
const ReactRefreshPlugin = require('@pmmmwh/react-refresh-webpack-plugin');
const { CleanWebpackPlugin } = require('clean-webpack-plugin');
const CopyPlugin = require('copy-webpack-plugin');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');
const detectPort = require('detect-port');

const { htmlPath, webPublicPath } = require('./path.js');

/**
 * @typedef {Object} Params
 * @property {string} outputPath - webpack打包输出路径
 * @property {string} contextPath - webpack打包上下文路径
 * @property {string} entrypoint - webpack打包入口文件
 * @property {string} prodSourceMap - webpack打包生产环境sourceMap
 * @property {string} version - webpack打包版本号
 * */

/**
 * @param {Params} params
 * @returns {*}
 * */
const devServerConfig = async (params) => {
    const DEFAULT_PORT = parseInt(process.env.PORT, 10) || 8080;

    // 检测端口是否占用
    const PORT = await detectPort(DEFAULT_PORT);

    return {
        entry: {},
        output: {
            publicPath: '',
            path: params.outputPath,
        },
        devServer: {
            static: {
                directory: params.outputPath,
            },
            hot: true,
            open: true,
            port: PORT,
            allowedHosts: 'all',
        },

        plugins: [new CleanWebpackPlugin()],
    };
};

/**
 * @param {Params} params
 * @returns {*}
 * */
const mainConfig = (params) => (env, argv) => {
    const isProd = argv.mode === 'production';

    const appWebpackConfig = makeConfig(env, argv, {
        version: params.version,
    });

    return {
        name: 'dreamviewWebInit',
        ...appWebpackConfig,
        target: 'web',
        context: params.contextPath,
        entry: params.entrypoint,
        output: {
            publicPath: 'auto',
            path: params.outputPath,
            filename: isProd ? '[name].[contenthash].js' : '[name].js',
            chunkFilename: isProd ? '[name].[contenthash].js' : '[name].js',
        },
        devtool: isProd ? params.prodSourceMap : 'source-map',
        resolve: {
            extensions: ['.ts', '.tsx', '.js', '.jsx', '.less', '.css', '.json'],
        },
        plugins: [
            ...(appWebpackConfig.plugins ?? []),
            new DefinePlugin({
                __PLATFORM__: JSON.stringify('web'),
            }),
            new ReactRefreshPlugin(),
            new CopyPlugin({
                patterns: [
                    {
                        from: webPublicPath,
                        to: params.outputPath,
                        filter: (resourcePath) => !/\.html$/.test(resourcePath),
                    },
                ],
            }),
            new HtmlWebpackPlugin({
                template: htmlPath,
            }),
            isProd &&
                new MiniCssExtractPlugin({
                    filename: isProd ? '[name].[contenthash].css' : '[name].css',
                }),
        ].filter(Boolean),
    };
};

module.exports = {
    devServerConfig,
    mainConfig,
};
