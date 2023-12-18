const path = require('path');
const fs = require('fs');

function getEntry() {
    const componentsSrc = path.resolve(__dirname, '../src/components/');

    const components = fs.readdirSync(componentsSrc);
    const result = {};
    [...components].forEach((component) => {
        result[`components/${component}/index`] = [component];
    });
    return result;
}

module.exports = getEntry;
