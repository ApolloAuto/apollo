const {mainConfig} = require('@dreamview/dreamview-web/config/webpackConfig.js');

export default {
    stories: ['../packages/dreamview-core/**/*.stories.@(js|jsx|ts|tsx|mdx)','../packages/dreamview-lang/**/*.stories.@(js|jsx|ts|tsx|mdx)','../packages/dreamview-theme/**/*.stories.@(js|jsx|ts|tsx|mdx)'],
    addons: ['@storybook/addon-links', '@storybook/addon-essentials', '@storybook/addon-interactions'],
    framework: {
        name: '@storybook/react-webpack5',
        options: {
            fastRefresh: true,
        },
    },
    webpackFinal: (config) => {
        const coreWebpackConfig = mainConfig(
            {
                mode: config.mode,
                version: '0.0.0',
            },
        )(undefined, {mode: config.mode});

        return {
            ...config,
            optimization: {
                ...config.optimization,
                ...coreWebpackConfig.optimization,
                minimize: false,
            },
            resolve: {
                ...coreWebpackConfig.resolve,
            },
            module: {
                rules: coreWebpackConfig.module.rules,
            },
            plugins: (config.plugins ?? []).concat(
                (coreWebpackConfig.plugins ?? []).filter(
                    (plugin) => ![ 'ProvidePlugin', 'ProgressPlugin', 'ESLintWebpackPlugin'].includes(plugin.constructor.name),
                ),
            ),
        };
    },
};
