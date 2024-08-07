module.exports = {
    extends: [require.resolve('@dreamview/eslint-config-dreamview')],
    globals: {
        __PLATFORM__: 'readonly',
        'process.env.NODE_ENV': 'readonly',
        self: 'readonly',
    },
    rules: {
        'jsx-a11y/click-events-have-key-events': 0,
        'jsx-a11y/no-static-element-interactions': 0,
    },
};
