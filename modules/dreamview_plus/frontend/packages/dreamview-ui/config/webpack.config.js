const webpack = require('webpack');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');
const postcssLoader = require('postcss-loader');
const path = require('path');
const NodePolyfillPlugin = require('node-polyfill-webpack-plugin');
const ProgressBarPlugin = require('progress-bar-webpack-plugin');
const getEntry = require('./entries');
const paths = require('./paths');

const appRoot = path.join(__dirname, '../');
const resolveApp = (relativePath) => path.resolve(appRoot, relativePath);

console.log('entry: ', resolveApp('./src/index.ts'));

module.exports = {
    mode: 'production',
    entry: getEntry(),
    // entry: paths.entryRoot,
    output: {
        path: paths.outDir,
        filename: '[name].js',
        library: 'dreamview-ui-library',
        libraryTarget: 'umd',
        umdNamedDefine: true,
        // libraryExport:"default"
    },
    resolve: {
        extensions: ['.js', '.jsx', '.ts', '.tsx', '.less'],
        // alias: {
        //     '@': resolveApp('src'),
        //     components: resolveApp('./src/components'),
        //     src: resolveApp('src'),
        // },
        // fallback: {
        //     os: require.resolve('os-browserify/browser'),
        //     path: require.resolve('path-browserify'),
        //     crypto: require.resolve('crypto-browserify'),
        //     zlib: require.resolve('browserify-zlib'),
        //     stream: require.resolve('stream-browserify'),
        //     https: require.resolve('https-browserify'),
        //     http: require.resolve('stream-http'),
        //     vm: require.resolve('vm-browserify'),
        //     querystring: require.resolve('querystring-es3'),
        //     tty: require.resolve('tty-browserify'),
        // },
    },
    module: {
        rules: [
            {
                // test: /\.(js|jsx|ts|tsx)$/,
                test: /\.(js|jsx)$/,
                exclude: /node_modules/,
                use: [
                    // 'thread-loader',
                    {
                        loader: 'babel-loader',
                        options: {
                            presets: ['@babel/preset-env', '@babel/preset-react', '@babel/preset-typescript'],
                            plugins: [
                                // @ 装饰器配置
                                ['@babel/plugin-proposal-decorators', { legacy: true }],
                                // 可选链
                                '@babel/plugin-proposal-optional-chaining',
                                [
                                    '@babel/plugin-transform-modules-commonjs',
                                    {
                                        allowTopLevelThis: true,
                                    },
                                ],
                            ],
                        },
                    },
                ],
            },
            {
                test: /\.(ts|tsx)$/,
                exclude: /node_modules/,
                use: [
                    // 'thread-loader',
                    {
                        loader: 'babel-loader',
                        options: {
                            presets: ['@babel/preset-env', '@babel/preset-react', '@babel/preset-typescript'],
                            plugins: [
                                // @ 装饰器配置
                                ['@babel/plugin-proposal-decorators', { legacy: true }],
                                // 可选链
                                '@babel/plugin-proposal-optional-chaining',
                                [
                                    '@babel/plugin-transform-modules-commonjs',
                                    {
                                        allowTopLevelThis: true,
                                    },
                                ],
                            ],
                        },
                    },
                    'ts-loader',
                ],
            },
            // {
            //     test: /\.less$/,
            //     use: [
            //         MiniCssExtractPlugin.loader,
            //         {
            //             loader: 'css-loader',
            //         },
            //         {
            //             loader: postcssLoader,
            //             options: {
            //                 postcssOptions: {
            //                     ident: 'postcss',
            //                     plugins: ['postcss-flexbugs-fixes', 'postcss-preset-env', 'postcss-normalize'],
            //                 },
            //             },
            //         },
            //         {
            //             loader: 'less-loader',
            //             options: {
            //                 sourceMap: true,
            //                 javascriptEnabled: true,
            //             },
            //         },
            //     ],
            //     sideEffects: true,
            // },
            {
                test: /\.less$/,
                use: [
                    // 'style-loader',
                    {
                        loader: 'css-loader',
                    },
                    {
                        loader: 'postcss-loader',
                        options: {
                            postcssOptions: {
                                ident: 'postcss',
                                plugins: ['postcss-flexbugs-fixes', 'postcss-preset-env', 'postcss-normalize'],
                            },
                        },
                    },
                    {
                        loader: 'less-loader',
                        options: {
                            sourceMap: true,
                            lessOptions: {
                                javascriptEnabled: true,
                            },
                        },
                    },
                ],
                sideEffects: true,
            },
            {
                test: /\.(png|jpe?g|mp4|mov|gif)$/i,
                type: 'asset',
                generator: {
                    filename: 'assets/[hash][ext][query]',
                },
            },
            {
                test: /\.svg$/,
                use: [
                    {
                        loader: 'babel-loader',
                    },
                    {
                        loader: '@svgr/webpack',
                        options: {
                            babel: false,
                            icon: true,
                        },
                    },
                ],
            },
        ],
    },
    plugins: [
        new webpack.DefinePlugin({}),
        new MiniCssExtractPlugin({
            filename: 'css/[name].[contenthash:8].css',
            chunkFilename: 'css/[name].[contenthash:8].chunk.css',
        }),
        new NodePolyfillPlugin(),
        new ProgressBarPlugin({
            format: 'build [:bar] :percent (:elapsed seconds)',
            clear: false,
        }),
    ],
    externals: {
        react: 'react',
        'react-dom': 'react-dom',
    },
    target: 'web',
};
