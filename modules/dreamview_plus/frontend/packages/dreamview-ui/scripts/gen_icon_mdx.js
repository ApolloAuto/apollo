const fs = require('fs');
const path = require('path');

const ICONS_DIR = path.resolve(__dirname, '../src/IconPark/Light/svg');
const ICONS_DRAK_DIR = path.resolve(__dirname, '../src/IconPark/Drak/svg');
const OUTPUT_FILE = path.resolve(__dirname, '../src/mdxs/IconList.mdx');
const { camelCase, upperFirst } = require('lodash');

const generateIconListMDX = () => {
    const iconFiles = fs.readdirSync(ICONS_DIR);

    const iconsLight = iconFiles.map((file) => {
        const iconName = file.replace('.svg', '');
        return { name: upperFirst(camelCase(iconName)) };
    });

    const importString = `import IconPark from '../IconPark';
import { Canvas, Meta } from '@storybook/blocks';
import { message } from 'antd';
`;

    const cols = 5;
    const iconTable = `<table style={{width: '100%'}}><tbody>${Array.from({
        length: Math.ceil(iconsLight.length / cols),
    })
        .map((_, i) => {
            const rowIcons = iconsLight.slice(i * cols, i * cols + cols);
            return `<tr>${rowIcons
                .map(
                    (icon) =>
                        `<td align="center" style={{paddingTop: 20}}>
  <IconPark name="${icon.name}" width="16" height="16"/>
  <br/> <a style={{ cursor: 'pointer' }} onClick={() => {
    var copyipt = document.createElement("input");
    var text = '<IconPark name="${icon.name}" />';
    copyipt.setAttribute("value", text);
    document.body.appendChild(copyipt);
    copyipt.select();
    document.execCommand("copy");
    document.body.removeChild(copyipt);
    message.success('复制成功');
  }}><IconPark name="${icon.name}" /></a>
</td>`,
                )
                .join('')}</tr>`;
        })
        .join('')}</tbody></table>`;

    const mdxContent = `${importString}

<Meta title="Icon 组件列表" />

# Icon 组件列表

以下是一些示例的 Icon 组件：

${iconTable}
`;

    fs.writeFileSync(OUTPUT_FILE, mdxContent, 'utf8');

    console.log(`Generated Icon List MDX file: ${OUTPUT_FILE}`);
};

generateIconListMDX();
