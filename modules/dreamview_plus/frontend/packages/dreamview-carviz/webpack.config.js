const webpack = require('webpack');
const TerserPlugin = require('terser-webpack-plugin');
const HTMLPlugin = require('html-webpack-plugin');
const CopyWebpackPlugin = require('copy-webpack-plugin');

const isDev = process.env.NODE_ENV === 'development';

module.exports = {
    target: 'web',
    devServer: {
        port: 9000,
        host: '0.0.0.0',
        hot: true,
    },
    entry: {
        index: './src/index.ts',
        'index.min': './src/index.ts',
    },
    output: {
        filename: '[name].js',
        library: 'carviz',
        libraryTarget: 'umd',
    },
    resolve: {
        extensions: ['.js', '.ts'],
    },
    mode: 'none',
    optimization: {
        minimize: true,
        minimizer: [
            new TerserPlugin({
                test: /\.min.js/, // 提供一个正则，表示符合有min.js的就进行压缩
            }),
        ],
    },
    module: {
        rules: [
            {
                test: /\.(ts|tsx)$/,
                exclude: '/node-modules/',
                loader: 'babel-loader',
            },
            // {
            //     test: /\.(js|jsx|ts|tsx)$/,
            //     exclude: /node_modules/,
            //     use: [
            //         {
            //             loader: 'babel-loader',
            //             options: {
            //                 presets: ['@babel/preset-env', '@babel/preset-typescript'],
            //                 plugins: [
            //                     '@babel/plugin-proposal-optional-chaining',
            //                     '@babel/plugin-proposal-class-properties',
            //                 ],
            //             },
            //         },
            //     ],
            // },
            {
                test: /\.(jpe?g|png|gif)$/i,
                use: {
                    loader: 'file-loader',
                    options: {
                        name: '[name][hash:8].[ext]',
                        outputPath: 'assets/images',
                    },
                },
            },
            {
                test: /\.(mtl|obj|gltf|drc|glb|fbx)$/,
                use: {
                    loader: 'file-loader',
                    options: {
                        name: '[name][hash:8].[ext]',
                        outputPath: 'assets/models',
                    },
                },
            },
            {
                test: /\.(otf|eot|woff2?|ttf|svg|json)$/i,
                use: {
                    loader: 'file-loader',
                    options: {
                        name: '[name][hash:8].[ext]',
                        outputPath: 'assets/fonts',
                    },
                },
            },
        ],
    },
    plugins: [
        new webpack.DefinePlugin({
            'process.env': {
                NODE_ENV: isDev ? '"development"' : '"production"',
            },
        }),
        new HTMLPlugin({
            template: './index.html',
        }),
        new CopyWebpackPlugin({
            patterns: [
                {
                    from: './assets/fonts',
                    to: 'assets/fonts',
                },
            ],
        }),
    ],
};
