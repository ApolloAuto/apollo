module.exports = {
    extends: [require.resolve('@dreamview/eslint-config-dreamview')],
    rules: {
        'object-curly-newline': 'off',
        'arrow-body-style': 'off',
        '@typescript-eslint/no-shadow': 'off',
        'no-continue': 'off',
        'no-param-reassign': 'off',
    },
    // "env": {
    //     "browser": true,
    //     "es6": true,
    //     "node": true
    // },
    // "extends": [
    //     // "eslint:recommended",
    //     // "plugin:@typescript-eslint/recommended",
    // ],
    // "parser": "@typescript-eslint/parser",
    // "parserOptions": {
    //     "sourceType": "module"
    // },
    // "plugins": ["@typescript-eslint"],
    // "rules": {
    //     "indent": [
    //         "error",
    //         4
    //     ],
    //     "linebreak-style": [
    //         "error",
    //         "unix"
    //     ],
    //     "quotes": [
    //         "error",
    //         "single"
    //     ],
    //     "semi": [
    //         "error",
    //         "always"
    //     ],
    //     "no-console": "off",
    //     "max-classes-per-file": ["error", 2],
    // }
};
