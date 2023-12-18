module.exports = {
    extends: [require.resolve('@dreamview/eslint-config-dreamview')],
    env: {
        worker: true,
        browser: true,
        node: true,
    },
    globals: {
        self: 'writable',
    },
    rules: {
        'no-restricted-globals': 'off',
    },
};
