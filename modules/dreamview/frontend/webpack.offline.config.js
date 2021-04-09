'use strict'

const path = require("path");
const webpack = require("webpack");
const HtmlWebpackPlugin = require("html-webpack-plugin");
const FaviconsWebpackPlugin = require("favicons-webpack-plugin");
const ProgressBarPlugin = require("progress-bar-webpack-plugin");
const CopyWebpackPlugin = require('copy-webpack-plugin');
// const BundleAnalyzerPlugin = require('webpack-bundle-analyzer').BundleAnalyzerPlugin;

module.exports = {
    context: path.join(__dirname, "src"),

    entry: {
        offline: "./offline.js",
    },

    output: {
        path: path.join(__dirname, "dist_offline"),
        filename: "[name].bundle.js",
        publicPath: "/offlineview/",
    },

    devtool: "cheap-source-map",

    resolve: {
        // Files with the following extensions here can be "import"
        // without the extension.
        //
        // Needs ".json" and ".scss" for Grommet.
        extensions: [".wepack.js", ".web.js",
                     ".jsx", ".js",
                     ".json",
                     ".scss", ".css",
                     ".png", ".svg"],
        alias: {
            store: path.resolve(__dirname, "src/store"),
            styles: path.resolve(__dirname, "src/styles"),
            components: path.resolve(__dirname, "src/components"),
            utils: path.resolve(__dirname, "src/utils"),
            renderer: path.resolve(__dirname, "src/renderer"),
            assets: path.resolve(__dirname, "assets"),
            proto_bundle: path.resolve(__dirname, "proto_bundle"),
        }
    },

    module: {
        rules: [
            {
                test: /\.js$/,
                exclude: /node_modules/,
                use: [
                    {
                        loader: "babel-loader",
                        options: {
                            // Use es2015 so that async/await are available.
                            presets: ["es2015", "stage-0", "react"],
                            // Enable decorators for mobx.
                            plugins: ["transform-decorators-legacy",
                                      "transform-runtime"],
                        }
                    }
                ],
            }, {
                // Run eslint. See .eslinrc for configuration.
                test: /\.js$/,
                enforce: "pre",
                loader: "eslint-loader",
            }, {
                test: /\.yml$/,
                exclude: /node_modules/,
                use: [
                    { loader: "json-loader" },
                    { loader: "yaml-loader" }
                ],
            }, {
                // Object models and materials.
                test: /\.mtl$|\.obj$/,
                exclude: /node_modules/,
                use: [{
                    loader: "file-loader",
                }],
            }, {
                test: require.resolve("three/examples/js/loaders/MTLLoader.js"),
                use: "imports-loader?THREE=three"
            }, {
                test: require.resolve("three/examples/js/loaders/OBJLoader.js"),
                use: "imports-loader?THREE=three"
            }, {
                test: require.resolve("three/examples/js/controls/OrbitControls.js"),
                use: "imports-loader?THREE=three"
            }, {
                // Load the images. They goes through image-webpack-loader
                // first, and then file-loader.
                //
                // Now you can import images just like js.
                test: /\.(png|jpe?g|svg|mp4|mov|gif)$/i,
                use: [
                    {
                        loader: "file-loader",
                        options: {
                            name: 'assets/[hash:base64:55].[ext]',
                        }
                    }, {
                        loader: "image-webpack-loader",
                        options: {
                            pngquant: {
                                quality: "65-90",
                                speed: 4,
                            },
                            mozjpeg: {
                                progressive: true,
                            }
                        }
                    }
                ]
            }, {
                // This is to apply the following style loaders in (reverse) order.
                // Grommet scss files needs to be processed this way.
                test: /\.scss$/,
                use: [
                    {
                        loader: "style-loader",
                    }, {
                        loader: "css-loader",
                    }, {
                        loader: "sass-loader",
                        options: {
                            includePaths: [
                                "./node_modules",
                            ]
                        }
                    }
                ],
            }, {
                // For font-awesome (woff)
                test: /\.woff(2)?(\?v=[0-9]\.[0-9]\.[0-9])?$/,
                use: [
                    {
                        loader: "url-loader",
                        options: {
                            limit: 10000,
                            mimetype: "application/font-woff",
                        }
                    }
                ],
            }, {
                // For font-awesome (ttf)
                test: /\.(ttf|eot|svg)(\?v=[0-9]\.[0-9]\.[0-9])?$/,
                loader: "file-loader",
            }, {
                test: /webworker\.js$/,
                use: [
                {
                    loader: 'worker-loader',
                    options: {
                        name: 'worker.bundle.js'
                    },
                },
                {
                    loader: "babel-loader",
                    options: {
                        presets: ["es2015"],
                    }
                }]
            },
        ]
    },

    plugins: [
        // Show compilation progress bar.
        new ProgressBarPlugin(),

        // Generate the actual html from the hbs.
        new HtmlWebpackPlugin({
            template: "./index.hbs",
            // Include only the app. Do not include the service worker.
            chunks: ["offline"]
        }),
        new FaviconsWebpackPlugin("./favicon.png"),
        new CopyWebpackPlugin([
            {
                from: '../node_modules/three/examples/fonts',
                to: 'fonts',
            }
        ]),
        // As of moment 2.18, all locales are bundled together with the core library
        // use the IgnorePlugin to stop any locale being bundled with moment:
        new webpack.IgnorePlugin(/^\.\/locale$/, /moment$/),

        // Configure global constant at "compile" time
        // to allow different behavior between offline or realtime
        new webpack.DefinePlugin({
            OFFLINE_PLAYBACK: JSON.stringify(true),
        }),

        // new webpack.optimize.UglifyJsPlugin({
        //     compress: {warnings: false},
        //     output: {comments: false},
        //     sourceMap: true,
        // }),

        // new BundleAnalyzerPlugin(),
    ],

    devServer: {
        contentBase: path.join(__dirname, "src"),
        host: '0.0.0.0',
        port: 8080,
        // Ask react-router instead of the server to handle routes.
        historyApiFallback: true,
        disableHostCheck: true
    },

    // Disable the warning of large size bundle files.
    performance: {
        hints: false
    },
};
