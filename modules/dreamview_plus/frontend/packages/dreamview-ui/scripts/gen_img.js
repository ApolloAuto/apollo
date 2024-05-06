const fs = require('fs');
const path = require('path');

const RAW_DIR = path.resolve(__dirname, '../src/components/ImagePrak/light');
const OUTPUT_FILE = path.resolve(__dirname, '../src/components/ImagePrak/type.ts');

const LIGHT_FILE = path.resolve(__dirname, '../src/components/ImagePrak/light');
const LIGHT_OUTPUT = path.resolve(__dirname, '../src/components/ImagePrak/light/index.ts');

const DRAK_FILE = path.resolve(__dirname, '../src/components/ImagePrak/drak');
const DRAK_OUTPUT = path.resolve(__dirname, '../src/components/ImagePrak/drak/index.ts');

function getImgFiles(filepath) {
    return fs
        .readdirSync(filepath)
        .filter((item) => ['png', 'gif', 'jpg', 'jpeg'].some((fileType) => item.endsWith(fileType)));
}

function toModules(filePath, outPut) {
    const imgFiles = getImgFiles(filePath);
    const typesString = imgFiles.map((item) => `${item.replace(/\..+/, '')}`).join(',\n');
    const importString = imgFiles.map((item) => `import ${item.replace(/\..+/, '')} from './${item}'`).join(';\n');
    const fileContent = `/* eslint-disable indent */
/* eslint-disable prettier/prettier */
/* eslint-disable camelcase */
${importString}

export default {
    ${typesString}
};
`;
    fs.writeFileSync(outPut, fileContent, 'utf8');
}

function DoS() {
    const imgFiles = getImgFiles(RAW_DIR);
    const typesString = imgFiles.map((item) => `'${item.replace(/\..+/, '')}'`).join('|\n');
    const fileContent = `/* eslint-disable prettier/prettier */
export type IImageName =
${typesString};
`;
    fs.writeFileSync(OUTPUT_FILE, fileContent, 'utf8');
}

DoS();
toModules(LIGHT_FILE, LIGHT_OUTPUT);
toModules(DRAK_FILE, DRAK_OUTPUT);
