const fs = require('fs');
const path = require('path');

const ICONS_DIR = path.resolve(__dirname, '../src/icons/components');
const OUTPUT_FILE = path.resolve(__dirname, '../src/mdxs/IconList.mdx');

const generateIconListMDX = () => {
    const iconFiles = fs.readdirSync(ICONS_DIR).filter((file) => file.endsWith('.tsx'));

    const icons = iconFiles.map((file) => {
        const iconName = file.replace('.tsx', '');
        return { name: iconName };
    });

    const importString = `import { ${icons
        .map((icon) => `Icon${icon.name}`)
        .join(
            ', ',
        )} } from '../icons';\nimport { Canvas, Meta } from '@storybook/blocks';\nimport { message } from 'antd';`;

    const cols = 5;
    const iconTable = `<table><tbody>${Array.from({ length: Math.ceil(icons.length / cols) })
        .map((_, i) => {
            const rowIcons = icons.slice(i * cols, i * cols + cols);
            return `<tr>${rowIcons
                .map(
                    (icon) =>
                        `<td align="center" style={{paddingTop: 20}}>
  <Icon${icon.name} width="16" height="16"/>
  <br/> <a style={{ cursor: 'pointer' }} onClick={() => {
    var copyipt = document.createElement("input");
    var text = '<Icon${icon.name} />';
    copyipt.setAttribute("value", text);
    document.body.appendChild(copyipt);
    copyipt.select();
    document.execCommand("copy");
    document.body.removeChild(copyipt);
    message.success('复制成功');
  }}>< Icon${icon.name} /></a>
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
