const webpack = require('webpack');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');
const postcssLoader = require('postcss-loader');
const path = require('path');
const getEntry = require('./entries');

const appRoot = path.join(__dirname, '../');
const resolveApp = (relativePath) => path.resolve(appRoot, relativePath);

module.exports = {
    mode: 'production',
    entry: getEntry(),
    resolve: {
        extensions: ['.js', '.jsx', '.ts', '.tsx', '.less'],
        alias: {
            '@': resolveApp('src'),
            components: resolveApp('./src/components'),
            src: resolveApp('src'),
        },
    },
    module: {
        rules: [
            {
                test: /\.(js|jsx|ts|tsx)$/,
                exclude: /node_modules/,
                use: [
                    'thread-loader',
                    {
                        loader: 'babel-loader',
                        options: {
                            presets: [
                                '@babel/preset-env',
                                '@babel/preset-react',
                                '@babel/preset-typescript',
                            ],
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
                test: /\.less$/,
                use: [
                    MiniCssExtractPlugin.loader,
                    {
                        loader: 'css-loader',
                    },
                    {
                        loader: postcssLoader,
                        options: {
                            postcssOptions: {
                                ident: 'postcss',
                                plugins: [
                                    'postcss-flexbugs-fixes',
                                    'postcss-preset-env',
                                    'postcss-normalize',
                                ],
                            },

                        },
                    },
                    {
                        loader: 'less-loader',
                        options: {
                            sourceMap: true,
                            javascriptEnabled: true,
                        },
                    },
                ],
                sideEffects: true,
            },
            {
                test: /\.(png|jpe?g|svg|mp4|mov|gif)$/i,
                type: 'asset',
                generator: {
                    filename: 'assets/[hash][ext][query]',
                },
            },
        ],
    },
    plugins: [
        new webpack.DefinePlugin({}),
        new MiniCssExtractPlugin({
            filename: 'css/[name].[contenthash:8].css',
            chunkFilename: 'css/[name].[contenthash:8].chunk.css',
        }),
    ],
    externals: {
        react: 'react',
        'react-dom': 'react-dom',
    },
};
