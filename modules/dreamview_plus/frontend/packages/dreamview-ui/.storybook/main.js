/** @type { import('@storybook/react-webpack5').StorybookConfig } */
const postcssLoader = require('postcss-loader');

const config = {
  stories: ['../src/**/*.mdx', '../src/**/*.stories.@(js|jsx|ts|tsx)'],
  addons: [
    '@storybook/addon-links',
    '@storybook/addon-essentials',
    '@storybook/addon-interactions',
  ],
  framework: {
    name: '@storybook/react-webpack5',
    options: {},
  },
  docs: {
    autodocs: 'tag',
  },
  webpackFinal: async (config) => {
    config.module.rules
      .filter(rule => rule.test && rule.test.test('.svg'))
      .forEach(rule => rule.exclude = /\.svg$/i);

      config.module.rules.push({
        test: /\.(ts|tsx)$/,
        loader: require.resolve('babel-loader'),
        options: {
          presets: [
            '@babel/preset-env',
            '@babel/preset-react',
            '@babel/preset-typescript',
          ],
        },
      }, {
        test: /\.less$/,
        use: [
          'style-loader',
            {
                loader: 'css-loader',
            },
            {
              loader: 'postcss-loader',
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
                lessOptions: {
                  javascriptEnabled: true,
                },
              },
            },
        ],
        sideEffects: true,
      }, {
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
      });
    config.resolve.extensions.push('.ts', '.tsx');
    return config;
  },
};
export default config;
