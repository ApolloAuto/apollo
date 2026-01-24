const path = require('path');

const makeConfig = require('@dreamview/dreamview-core/config/webpack.section.js');
const { DefinePlugin } = require('webpack');
const ReactRefreshPlugin = require('@pmmmwh/react-refresh-webpack-plugin');
const { CleanWebpackPlugin } = require('clean-webpack-plugin');
const CopyPlugin = require('copy-webpack-plugin');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');
const detectPort = require('detect-port');
const { GenerateSW } = require('workbox-webpack-plugin');
const ModuleFederationPlugin = require('webpack/lib/container/ModuleFederationPlugin');

const { htmlPath, webPublicPath, packageJson } = require('./path.js');

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
            proxy: {
                '/proto': {
                    target: 'http://localhost:8888',
                    pathRewrite: { '^/proto': '/proto' },
                    changeOrigin: true,
                },
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
            new CleanWebpackPlugin({
                cleanOnceBeforeBuildPatterns: [params.outputPath],
            }),
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
            // todo： 暂时去除service worker注册
            // isProd &&
            //     new GenerateSW({
            //         clientsClaim: true,
            //         skipWaiting: true,
            //         runtimeCaching: [
            //             {
            //                 // 匹配 localhost:3000 下的所有请求
            //                 urlPattern: new RegExp(
            //                     '^http://apollo-studio-staging-public.cdn.bcebos.com/dreamview/panel.*',
            //                 ),
            //                 handler: 'NetworkFirst', // 或者使用 'CacheFirst', 'StaleWhileRevalidate' 等策略
            //                 options: {
            //                     cacheName: 'localhost-resources',
            //                     networkTimeoutSeconds: 3, // 可选，设置网络请求超时时间
            //                     cacheableResponse: {
            //                         statuses: [0, 200], // 缓存状态码为 0 和 200 的响应
            //                     },
            //                     expiration: {
            //                         // 可选，配置缓存过期策略
            //                         maxEntries: 100, // 缓存的最大条目数
            //                         maxAgeSeconds: 30 * 24 * 60 * 60, // 缓存的最大周期（例如，30天）
            //                     },
            //                 },
            //             },
            //         ],
            //     }),
            new ModuleFederationPlugin({
                // 基座应用
                name: 'host',
                shared: {
                    // 共享模块
                    react: { singleton: true, eager: true, requiredVersion: packageJson.dependencies.react },
                    'react-dom': {
                        singleton: true,
                        eager: true,
                        requiredVersion: packageJson.dependencies['react-dom'],
                    },
                },
            }),
        ].filter(Boolean),
    };
};

module.exports = {
    devServerConfig,
    mainConfig,
};
