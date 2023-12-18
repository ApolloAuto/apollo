// 使用nodejs将当前上级目录下的.web_build目录下的文件拷贝到frontend下的dist目录
const fs = require('fs-extra');
const path = require('../config/paths.js');

const copy = async () => {
    const src = path.outputPath;
    const dest = `${path.frontendPath}/dist`;
    await fs.copy(src, dest);
};

copy().then(
    () => {
        console.log('copy success');
    },
    (err) => {
        console.log('copy failed', err);
    },
);
