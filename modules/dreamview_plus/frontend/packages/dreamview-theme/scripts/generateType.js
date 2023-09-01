import path from 'path';
import fs from 'fs';
import allColors from '../tokens/baseToken/allColors';

function writeFile(vars, outputPath) {
    const varsToWrite = vars.map(
        (key) => `    ${key}Base: string;
    ${key}1: string;
    ${key}2: string;
    ${key}3: string;
    ${key}4: string;
    ${key}5: string;
    ${key}6: string;
    ${key}7: string;
    ${key}8: string;
    ${key}9: string;
    ${key}10: string;
`,
    ).join(`
`);

    const types = `export type IDreamviewColors = {
${varsToWrite.replace(/\n$/, '')}
};
`;

    fs.writeFileSync(outputPath, types);
}

const outputDir = path.join(__dirname, '../tokens/baseToken');

const filename = 'type.ts';
const colors = Object.keys(allColors);

writeFile(colors, path.join(outputDir, filename));
