const path = require('path');
const CircularDependencyPlugin = require('circular-dependency-plugin');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');
const postcssFlexbugsFixes = require('postcss-flexbugs-fixes');
const postcssPresetEnv = require('postcss-preset-env');
const postcssNormalize = require('postcss-normalize');
const ESLintPlugin = require('eslint-webpack-plugin');
const ProgressBarPlugin = require('progress-bar-webpack-plugin');
const webpack = require('webpack');

/**
 *
 * @param {*} mode - webpack打包模式
 * @param {*} argv - webpack打包参数
 * @param {version: string} option
 * @returns
 */
module.exports = function makeConfig(mode, argv, option) {
    const isProd = argv.mode === 'production';
    const isDev = argv.mode === 'development';

    const { version } = option;

    const cssRegex = /\.css$/;
    const lessRegex = /\.less$/;

    const getStyleLoaders = (cssOptions, preProcessor = '', preProcessorOption = {}) => {
        const loaders = [
            isDev && require.resolve('style-loader'),
            isProd && {
                loader: MiniCssExtractPlugin.loader,
                options: { publicPath: '/build/' },
            },
            {
                loader: require.resolve('css-loader'),
                options: cssOptions,
            },
            {
                loader: require.resolve('postcss-loader'),
                options: {
                    postcssOptions: {
                        ident: 'postcss',
                        plugins: [
                            postcssFlexbugsFixes,
                            postcssPresetEnv({
                                autoprefixer: {
                                    flexbox: 'no-2009',
                                },
                                browsers: 'last 2 versions',
                            }),
                            postcssNormalize(),
                        ],
                    },
                    sourceMap: isDev,
                },
            },
        ].filter(Boolean);
        if (preProcessor) {
            loaders.push('resolve-url-loader', {
                loader: require.resolve(preProcessor),
                options: {
                    sourceMap: isDev,
                    ...preProcessorOption,
                },
            });
        }
        return loaders;
    };

    return {
        resolve: {
            extensions: ['.ts', '.tsx', '.js', '.jsx', '.less', '.css', '.json'],
            alias: {
                '@src': path.join(__dirname, '../src'),
                '@components': path.join(__dirname, '../src/components'),
                three: path.resolve(__dirname, 'node_modules/three'),
            },
            fallback: {
                assert: require.resolve('assert'),
                buffer: require.resolve('buffer'),
            },
        },
        module: {
            rules: [
                {
                    test: /\.(js|jsx|ts|tsx)$/,
                    exclude: [/node_modules/, /storybook-stories.js/, /storybook-config-entry.js/],
                    use: [
                        {
                            loader: 'babel-loader',
                            options: {
                                presets: ['@babel/preset-env', '@babel/preset-react', '@babel/preset-typescript'],
                                plugins: [
                                    isDev && require.resolve('react-refresh/babel'),
                                    ['@babel/plugin-proposal-decorators', { legacy: true }],
                                ].filter(Boolean),
                            },
                        },
                    ],
                },
                {
                    test: cssRegex,
                    use: getStyleLoaders({
                        importLoaders: 1,
                        sourceMap: isDev,
                        modules: false,
                    }),
                    sideEffects: true,
                },
                {
                    test: lessRegex,
                    use: getStyleLoaders(
                        {
                            importLoaders: 3,
                            sourceMap: !isDev,
                            modules: false,
                        },
                        'less-loader',
                    ),
                    sideEffects: true,
                },
                {
                    // Load the images. They go through image-webpack-loader
                    // first, and then file-loader.
                    //
                    // Now you can import images just like js.
                    test: /\.(png|jpe?g|mp4|mov|gif)$/i,
                    type: 'asset',
                    generator: {
                        filename: 'assets/[hash][ext][query]',
                    },
                },
                {
                    // Object models and materials.
                    test: /\.mtl$|\.obj$/,
                    exclude: /node_modules/,
                    use: [
                        {
                            loader: 'file-loader',
                        },
                    ],
                },
                {
                    test: /\.svg$/,
                    use: [
                        {
                            loader: 'babel-loader',
                            options: {
                                presets: ['@babel/preset-env', '@babel/preset-react', '@babel/preset-typescript'],
                            },
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
        optimization: {
            removeAvailableModules: isProd,
        },
        plugins: [
            new CircularDependencyPlugin({
                exclude: /node_modules/,
                failOnError: true,
            }),
            new webpack.ProvidePlugin({
                // 使得可以在项目中使用以下变量，而不必显式地导入它们
                React: 'react',
            }),
            new webpack.DefinePlugin({
                SANDBOX_PATHNAME: JSON.stringify(''),
                DREAMVIEW_VERSION: JSON.stringify(version),
                'process.env.NODE_ENV': JSON.stringify(argv.mode),
            }),
            new ProgressBarPlugin({
                format: 'build [:bar] :percent (:elapsed seconds)',
                clear: false,
            }),
            // fixme: 可能导致热更新阻塞
            // new ESLintPlugin({
            //     failOnError: true,
            //     cache: true,
            //     fix: true,
            //     lintDirtyModulesOnly: true,
            //     threads: true,
            //     extensions: ['js', 'jsx', 'ts', 'tsx'],
            // }),
        ],
        stats: {
            warnings: false,
        },
        target: 'web',
        node: {
            __dirname: true,
            __filename: true,
        },
        // fixme: 每次出现阻塞调试，后期再完善
        // performance: {
        //     hints: 'warning',
        //     maxAssetSize: 200000,
        //     maxEntrypointSize: 400000,
        // },
    };
};
