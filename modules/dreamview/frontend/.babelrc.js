module.exports = {
    presets: [
        '@babel/preset-env',
        '@babel/preset-react'],
    plugins: [
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
    ]
}
