import util from '../util';
import * as lightTheme from '../../light';
import * as drakTheme from '../../drak';

export enum ISize {
    sm = 'sm',
    regular = 'regular',
    large = 'large',
    huge = 'huge',
}

const genereteTypography = (font: any, fontSize: string, fontWeight: number, fontFamily = 'PingFangSC-Regular') => ({
    fontSize,
    fontWeight,
    fontFamily,
    lineHeight: font.lineHeight.regular,
});

export type ITheme = 'light' | 'drak';

export const themeList = ['light', 'drak'] as ITheme[];

export const genTokens = (colors: typeof lightTheme.colors, font: typeof lightTheme.font) => ({
    // 所有可用的颜色
    colors,
    font,
    padding: {
        speace0: '0',
        speace: '8px',
        speace2: '16px',
        speace3: '24px',
    },
    margin: {
        speace0: '0',
        speace: '8px',
        speace2: '16px',
        speace3: '24px',
    },
    // 背景色
    backgroundColor: {
        main: colors.background1,
        mainLight: colors.background2,
        mainStrong: colors.background3,
        transparent: 'transparent',
    },
    // 定位层级
    zIndex: {
        app: 2000,
        drawer: 1200,
        modal: 1300,
        tooltip: 1500,
    },
    shadow: {
        level1: {
            top: '0px -10px 16px 0px rgba(12,14,27,0.1)',
            left: '-10px 0px 16px 0px rgba(12,14,27,0.1)',
            right: '10px 0px 16px 0px rgba(12,14,27,0.1)',
            bottom: '0px 10px 16px 0px rgba(12,14,27,0.1)',
        },
    },
    // 分割线
    divider: {
        color: {
            regular: colors.divider1,
            light: colors.divider2,
            strong: colors.divider3,
        },
        width: {
            sm: 1,
            regular: 1,
            large: 2,
        },
    },
    // 边框
    border: {
        width: '1px',
        borderRadius: {
            sm: 4,
            regular: 6,
            large: 8,
            huge: 10,
        },
    },
    // 排版布局
    typography: {
        // 标题
        title: genereteTypography(font, font.size.large, font.weight.medium),
        // 字号比标题大一级
        title1: genereteTypography(font, font.size.huge, font.weight.medium),
        // 正文/内容
        content: genereteTypography(font, font.size.regular, font.weight.regular),
        // 辅助信息
        sideText: genereteTypography(font, font.size.sm, font.weight.regular),
    },
    // 过渡动画
    transitions: {
        easeIn: (property = 'all') => `${property} 0.25s cubic-bezier(0.4, 0, 1, 1)`,
        easeInOut: (property = 'all') => `${property} 0.25s cubic-bezier(0.4, 0, 0.2, 1)`,
        easeOut: (property = 'all') => `${property} 0.25s cubic-bezier(0.0, 0, 0.2, 1)`,
        sharp: (property = 'all') => `${property} 0.25s cubic-bezier(0.4, 0, 0.6, 1)`,
        duration: {
            shortest: 150,
            shorter: 200,
            short: 250,
            standard: 300,
            complex: 375,
            enteringScreen: 225,
            leavingScreen: 195,
        },
    },
});

export const light = {
    tokens: genTokens(lightTheme.colors, lightTheme.font),
    components: lightTheme.components,
    util,
};

export const drak = {
    tokens: genTokens(drakTheme.colors, drakTheme.font),
    components: drakTheme.components,
    util,
};

export type IDefaultTheme = typeof light;
