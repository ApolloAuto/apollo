'use strict';

const path = require('path');
const webpack = require('webpack');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const ProgressBarPlugin = require('progress-bar-webpack-plugin');
const CopyWebpackPlugin = require('copy-webpack-plugin');
const ESLintPlugin = require('eslint-webpack-plugin');
const ReactRefreshWebpackPlugin = require('@pmmmwh/react-refresh-webpack-plugin');
const FaviconsWebpackPlugin = require('favicons-webpack-plugin');
// const BundleAnalyzerPlugin = require('webpack-bundle-analyzer').BundleAnalyzerPlugin;

module.exports = (env, argv) => {

  const { mode } = argv;

  const isEnvDevelopment = mode === 'development';
  const isEnvProduction = mode === 'production';

  // style files regexes
  const cssRegex = /\.css$/;
  const sassRegex = /\.(scss|sass)$/;
  const lessRegex = /\.less$/;


  return {
    context: path.join(__dirname, 'src'),

    entry: {
      app: './app.js',
      parameters: path.join(__dirname, 'src/store/config/parameters.yml'),
    },

    output: {
      path: path.join(__dirname, 'dist'),
      filename: './[name].bundle.js',
      publicPath: '',
    },

    devtool: isEnvDevelopment ? 'inline-source-map' : 'hidden-source-map',

    resolve: {
      // Files with the following extensions here can be "import"
      // without the extension.
      //
      // Needs ".json" and ".scss" for Grommet.
      extensions: ['.webpack.js', '.web.js',
        '.jsx', '.js',
        '.json',
        '.scss', '.css',
        '.png', '.svg'],
      alias: {
        store: path.resolve(__dirname, 'src/store'),
        styles: path.resolve(__dirname, 'src/styles'),
        components: path.resolve(__dirname, 'src/components'),
        utils: path.resolve(__dirname, 'src/utils'),
        renderer: path.resolve(__dirname, 'src/renderer'),
        assets: path.resolve(__dirname, 'assets'),
        proto_bundle: path.resolve(__dirname, 'proto_bundle'),
      },
    },

    module: {
      rules: [
        {
          test: /\.(js|jsx)$/,
          exclude: /node_modules/,
          use: [
            {
              loader: 'babel-loader',
              options: {
                presets: [
                  '@babel/preset-env',
                  '@babel/preset-react'],
                plugins: [
                  isEnvDevelopment && require.resolve('react-refresh/babel'),
                  ['@babel/plugin-proposal-decorators', {
                    legacy: true,
                  }],
                  '@babel/plugin-proposal-class-properties',
                  '@babel/plugin-proposal-optional-chaining',
                  [
                    '@babel/plugin-transform-modules-commonjs',
                    {
                      allowTopLevelThis: true,
                    },
                  ],
                  [
                    'import',
                    {
                      libraryName: 'antd',
                      libraryDirectory: 'es',
                      style: true,
                    },
                    'antd',
                  ],
                ].filter(Boolean),
              },
            },
          ],
        },
        {
          // Object models and materials.
          test: /\.mtl$|\.obj$/,
          exclude: /node_modules/,
          use: [{
            loader: 'file-loader',
          }],
        },
        {
          test: require.resolve('three/examples/js/loaders/MTLLoader.js'),
          use: 'imports-loader?THREE=three',
        },
        {
          test: require.resolve('three/examples/js/loaders/OBJLoader.js'),
          use: 'imports-loader?THREE=three',
        },
        {
          test: require.resolve('three/examples/js/controls/OrbitControls.js'),
          use: 'imports-loader?THREE=three',
        },
        {
          // Load the images. They goes through image-webpack-loader
          // first, and then file-loader.
          //
          // Now you can import images just like js.
          test: /\.(png|jpe?g|svg|mp4|mov|gif)$/i,
          type: 'asset',
          generator: {
            filename: 'assets/[hash][ext][query]',
          },
        },
        {
          test: cssRegex,
          use: [
            {
              loader: 'style-loader',
            }, {
              loader: 'css-loader',
            }
          ]
        },
        {
          test: sassRegex,
          use: [
            {
              loader: 'style-loader',
            },
            {
              loader: 'css-loader',
              options: {
                modules: {
                  mode: 'global',
                },
              }
            },
            {
              loader: 'sass-loader',
              options: {
                sassOptions: {
                  includePaths: [
                    './node_modules',
                  ],
                },
              },
            },
          ],
        },
        {
          test: lessRegex,
          use: [
            {
              loader: 'style-loader',
            },
            {
              loader: 'css-loader',
            },
            {
              loader: 'less-loader',
              options: {
                lessOptions: {
                  javascriptEnabled: true,
                }
              }
            },
          ],
        },
        {
          // For font-awesome (woff)
          test: /\.woff(2)?(\?v=[0-9]\.[0-9]\.[0-9])?$/,
          use: [
            {
              loader: 'url-loader',
              options: {
                limit: 10000,
                mimetype: 'application/font-woff',
              },
            },
          ],
        },
        {
          // For font-awesome (ttf)
          test: /\.(ttf|eot|svg)(\?v=[0-9]\.[0-9]\.[0-9])?$/,
          loader: 'file-loader',
        },
        {
          test: /webworker\.js$/,
          use: [
            {
              loader: 'worker-loader',
              options: {
                filename: 'worker.bundle.js',
              },
            },
            {
              loader: 'babel-loader',
              options: {
                presets: ['@babel/preset-env'],
              },
            }],
        },
        {
          oneOf: [
            {
              test: /parameters.yml/,
              use: [
                {
                  loader: 'file-loader',
                  options: {
                    name: '[path][name].json',
                    context: 'src/store/config',
                    outputPath: '.', // the "dist" dir
                  },
                },
                {
                  loader: 'yaml-loader',
                  options: {
                    asJSON: true,
                  }
                },
              ]
            },
            {
              test: /\.yml$/,
              exclude: /node_modules/,
              use: [
                'json-loader',
                {
                  loader: 'yaml-loader',
                  options: {
                    asJSON: true,
                  }
                },
              ],
            },
          ]
        }
      ],
    },

    plugins: [
      // Show compilation progress bar.
      new ProgressBarPlugin({
        format: 'build [:bar] :percent (:elapsed seconds)',
        clear: false,
      }),
      isEnvDevelopment && new ReactRefreshWebpackPlugin(),
      new ESLintPlugin({
        fix: true,
        // 仅针对变更文件
        lintDirtyModulesOnly: true,
        extensions: ['js', 'jsx'],
      }),
      // Generate the actual html from the hbs.
      new HtmlWebpackPlugin({
        template: './index.hbs',
        // Include only the app. Do not include the service worker.
        chunks: ['app'],
      }),
      new FaviconsWebpackPlugin({
        logo: './favicon.png',
        cache: true,
        prefix: 'icons/',
      }),
      new CopyWebpackPlugin([
        {
          from: '../node_modules/three/examples/fonts',
          to: 'fonts',
        },
      ]),
      // As of moment 2.18, all locales are bundled together with the core library
      // use the IgnorePlugin to stop any locale being bundled with moment:
      new webpack.IgnorePlugin({
        resourceRegExp: /^\.\/locale$/,
        contextRegExp: /moment$/,
      }),

      new webpack.DefinePlugin({
        OFFLINE_PLAYBACK: JSON.stringify(false),
      }),

      // Uncomment me to analyze bundles
      // new BundleAnalyzerPlugin({
      //     analyzerMode: 'server',
      //     analyzerPort: '7777'
      // }),
    ].filter(Boolean),

    devServer: {
      client: {
        progress: true,
      },
      static: {
        directory: path.join(__dirname, 'src'),
        staticOptions: {},
        serveIndex: true,
        watch: true,
      },
      hot: true,
      open: true,
      compress: true,
      port: 8080,
      devMiddleware: {
        // debug devserver
        // writeToDisk: true,
      },
    },

    // Disable the warning of large size bundle files.
    performance: {
      hints: false,
    },
  };
};
