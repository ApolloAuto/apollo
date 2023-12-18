import { generate } from '@ant-design/colors';
import { IDreamviewColors } from './type';
import allColors from './allColors';

type IPresetPrimaryColors = typeof allColors;

const presetPrimaryColors: IPresetPrimaryColors = allColors;
type IColorName = keyof typeof presetPrimaryColors;

type ITheme = Record<IColorName, string[]>;

const presetPalettes: ITheme = {} as ITheme;
const presetDarkPalettes: ITheme = {} as ITheme;

(Object.keys(presetPrimaryColors) as IColorName[]).forEach((key) => {
    presetPalettes[key] = generate(presetPrimaryColors[key]);

    // dark presetPalettes
    presetDarkPalettes[key] = generate(presetPrimaryColors[key], {
        theme: 'dark',
        backgroundColor: '#141414',
    });
});

const toColors = (vars: ITheme) =>
    (Object.keys(presetPalettes) as IColorName[]).reduce(
        (result, key) => ({
            ...result,
            [`${key}Base`]: vars[key][5],
            [`${key}1`]: vars[key][0],
            [`${key}2`]: vars[key][1],
            [`${key}3`]: vars[key][2],
            [`${key}4`]: vars[key][3],
            [`${key}5`]: vars[key][4],
            [`${key}6`]: vars[key][5],
            [`${key}7`]: vars[key][6],
            [`${key}8`]: vars[key][7],
            [`${key}9`]: vars[key][8],
            [`${key}10`]: vars[key][9],
        }),
        {},
    );

export const dreamviewColors = {
    light: toColors(presetPalettes) as IDreamviewColors,
    dark: toColors(presetDarkPalettes) as IDreamviewColors,
};
