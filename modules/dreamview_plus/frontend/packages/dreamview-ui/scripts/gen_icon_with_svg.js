/* eslint-disable object-curly-newline */
/* eslint-disable complexity, no-console */
const fs = require('fs');
const path = require('path');
const mkdirp = require('mkdirp');
const rimraf = require('rimraf');
const svgson = require('svgson');
const { camelCase, upperFirst, flatten } = require('lodash');
const svgo = require('svgo');

const stringify = svgson.stringify;

const svgoConfig = (prefix) => ({
    multipass: true,
    removeViewBox: false,
    floatPrecision: 3,
    plugins: [
        {
            name: 'cleanupIDs',
            params: {
                remove: true,
                minify: true,
                prefix: `icon-${prefix}-`,
            },
        },
    ],
});

const ICON_PATTERN = /^(.+)\.svg$/;

function genPath(filePath) {
    const RAW_DIR = path.resolve(__dirname, `${filePath}/raw_svgs`);
    const SVG_DIR = path.resolve(__dirname, `${filePath}/svgs`);
    const ICON_TPL_DIR = path.resolve(__dirname, `${filePath}/icon_templates`);
    const MODULE_TPL = fs.readFileSync(path.join(ICON_TPL_DIR, 'component.tpl'), 'utf8');
    const EXPORT_TPL = fs.readFileSync(path.join(ICON_TPL_DIR, 'export.tpl'), 'utf8');
    const TYPE_TPL = fs.readFileSync(path.join(ICON_TPL_DIR, 'type.tpl'), 'utf8');
    const ICON_SRC = path.resolve(__dirname, filePath);

    return {
        RAW_DIR,
        SVG_DIR,
        ICON_TPL_DIR,
        MODULE_TPL,
        EXPORT_TPL,
        TYPE_TPL,
        ICON_SRC,
    };
}

function walkElement(el, { enter, leave }) {
    if (typeof enter === 'function') {
        enter(el);
    }
    if (el.children && el.children.length) {
        el.children.forEach((child) => {
            // eslint-disable-next-line no-param-reassign
            child.parentNode = el;
            walkElement(child, { enter, leave });
            // eslint-disable-next-line no-param-reassign
            delete child.parentNode;
        });
    }
    if (typeof leave === 'function') {
        leave(el);
    }
}

async function getSVGFiles(DIR, { keepSvgFill = false }) {
    return fs
        .readdirSync(DIR)
        .filter((file) => ICON_PATTERN.test(file))
        .map((file) => {
            const slug = file.replace(ICON_PATTERN, (_, $1) => $1);
            const content = fs.readFileSync(path.resolve(DIR, file), 'utf8');
            return {
                slug,
                content,
                keepSvgFill,
            };
        });
}

function getContextAttr(el, attr) {
    let node = el.parentNode;
    while (node) {
        if (node.attributes && node.attributes[attr]) {
            return node.attributes[attr];
        }

        node = node.parentNode;
    }
    return null;
}

async function normalizeSVG(content, file, keepSvgFill) {
    const { error, data } = await svgo.optimize(content, svgoConfig(file.replace(/\.svg$/, '')));
    if (error) {
        console.error(file, error);
        return;
    }

    const el = await svgson.parse(data);
    console.log(`Normalizing ${file}...`);
    const { attributes } = el;
    let { width, height } = attributes;
    const { viewBox } = attributes;

    if (!viewBox && !(width && height)) {
        console.error(file, "doesn't contain a valid size declaration.");
        console.error(width, height, viewBox);
    } else if (viewBox) {
        // has viewBox, override width/height
        [, width, height] = (viewBox.match(/0 0 (\d+) (\d+)/) || []).map((size) => parseInt(size, 10));
    } else {
        // no viewBox, use width/height
        attributes.viewBox = `0 0 ${width} ${height}`;
    }

    if (!keepSvgFill) {
        walkElement(el, {
            enter(node) {
                // eslint-disable-next-line no-shadow
                const { attributes } = node;

                delete attributes.class;

                const ctxFill = (getContextAttr(node, 'fill') || '').toLowerCase();
                const ctxStroke = (getContextAttr(node, 'stroke') || '').toLowerCase();
                const attrFill = (attributes.fill || '').toLowerCase();
                const attrStroke = (attributes.stroke || '').toLowerCase();
                const arrtNoParseFill = (attributes['fill-noparse'] || '').toLowerCase();

                // Check if current node is a 'mask'
                if (node.name === 'mask') {
                    // Set fill color to white
                    attributes.fill = 'white';
                    return;
                }

                if (attrFill) {
                    if (!ctxFill) {
                        if (attrFill !== 'none') {
                            attributes.fill = 'currentColor';
                            console.log(`  fill: ${attrFill} → currentColor`);
                        }
                    } else if (attrFill === ctxFill) {
                        delete attributes.fill;
                        console.log(`  fill: ${attrFill} → / (same as context)`);
                    } else if (attrFill !== 'none') {
                        attributes.fill = 'currentColor';
                        console.log(`  fill: ${attrFill} → currentColor (different from context)`);
                    }
                }

                if (attrStroke) {
                    if (!ctxStroke) {
                        const strokeIsNoteNone = attrStroke !== 'none';
                        if (strokeIsNoteNone) {
                            attributes.stroke = 'currentColor';
                            console.log(`  stroke: ${attrStroke} → currentColor`);
                        } else {
                            delete attributes.stroke;
                            console.log(`  stroke: ${attrStroke} → / (same as default)`);
                        }
                    } else if (attrStroke && attrStroke === ctxStroke) {
                        delete attributes.stroke;
                        console.log(`  stroke: ${attrStroke} → / (same as context)`);
                    } else if (attrStroke !== 'none') {
                        attributes.stroke = 'currentColor';
                        console.log(`  stroke: ${attrStroke} → currentColor (different from context)`);
                    }
                }

                if (arrtNoParseFill) {
                    attributes.fill = arrtNoParseFill;
                }
            },
        });
    }

    // eslint-disable-next-line consistent-return
    return {
        el,
        content: stringify(el),
        width,
        height,
    };
}

async function generate(paths) {
    rimraf.sync(paths.SVG_DIR);
    mkdirp.sync(paths.SVG_DIR);

    const iconsDir = path.join(paths.ICON_SRC, 'icons');
    const iconComponentsDir = path.join(iconsDir, 'components');
    rimraf.sync(iconsDir);
    mkdirp.sync(iconsDir);
    mkdirp.sync(iconComponentsDir);

    const iconSvgs = await getSVGFiles(paths.RAW_DIR, { keepSvgFill: false });
    const flattenIconSvgs = flatten(iconSvgs);
    Promise.all(
        flattenIconSvgs.map(async ({ slug, content, keepSvgFill }) => {
            const file = `${slug}.svg`;
            const { content: svg } = await normalizeSVG(content, file, keepSvgFill);

            fs.writeFileSync(path.join(paths.SVG_DIR, file), svg, 'utf8');

            const name = upperFirst(camelCase(slug));

            const moduleCode = paths.MODULE_TPL.replace(/\{type\}/g, 'Icon')
                .replace(/\{slug\}/g, slug)
                .replace(/\{name\}/g, name);

            fs.writeFileSync(path.join(iconComponentsDir, `${name}.tsx`), moduleCode, 'utf8');

            return { slug, name, file };
        }),
    )
        .then((iconInfos) => {
            const exportFile = iconInfos
                .map(({ slug, name, file }) => `${paths.EXPORT_TPL.replace(/\{name\}/g, name)}\n`)
                .join('');
            fs.writeFileSync(path.join(iconsDir, 'index.ts'), exportFile, 'utf8');
            fs.writeFileSync(path.join(iconsDir, 'type.ts'), paths.TYPE_TPL, 'utf8');
        })
        .catch((error) => {
            console.error('Error generating icons:', error);
        });
}
const drakFilePath = genPath('../src/iconDrak');
const lightFilePath = genPath('../src');

generate(drakFilePath);
generate(lightFilePath);
