import { dreamviewColors } from './colors';

const colors = dreamviewColors.light;

export enum ISize {
    sm = 'sm',
    regular = 'regular',
    large = 'large',
    huge = 'huge',
}

const colors1 = {
    transparent: 'transparent',
    // 品牌色
    brand1: '#579FF1',
    brand2: '#3288FA',
    brand3: '#1971E6',
    brand4: '#1252C0',
    // 辅助色-错误
    error: '#f75660',
    // 辅助色-警告
    warn: '#FF8D26',
    // 辅助色-成功
    success: '#1FCC4D',
    // 背景色
    backgroundMask: 'rgba(255,255,255,0.65)',
    backgroundHover: 'rgba(115, 193, 250, 0.65)',
    background1: '#282B36',
    background2: '#1A1D24',
    background3: '#0F1012',
    background4: '#343C4D',
    background5: '#2E323B',
    // 字阶色
    fontColor1: '#FFF',
    fontColor2: '#A6B5CC',
    fontColor3: '#808B9D',
    fontColor4: '#5D6573',
    fontColor5: '#40454D',
    // 分割线
    divider1: '#383C4D',
    divider2: '#383B45',
    divider3: '#252833',
};

const font = {
    reactive: {
        mainHover: '#fff',
        mainActive: '#5D6573',
        mainDisabled: colors.grey3,
    },
    color: {
        primary: colors.blueBase,
        success: colors.greenBase,
        warn: colors.orangeBase,
        error: colors.redBase,
        black: colors.greyBase,
        white: 'white',
        main: colors1.fontColor3,
        mainLight: colors1.fontColor2,
        mainStrong: colors1.fontColor1,
    },
    size: {
        sm: '12px',
        regular: '14px',
        large: '16px',
        huge: '18px',
    },
    weight: {
        light: 300,
        regular: 400,
        medium: 500,
        semibold: 700,
    },
    lineHeight: {
        // 密集
        dense: 1.4,
        // 常规
        regular: 1.5714,
        // 稀疏
        sparse: 1.8,
    },
    fontFamily:
        '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,"Helvetica Neue", Arial, "PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", sans-serif',
};

const genereteTypography = (fontSize: string, fontWeight: number, fontFamily = 'PingFangSC-Regular') => ({
    fontSize,
    fontWeight,
    fontFamily,
    lineHeight: font.lineHeight.regular,
});

const defaultTokens = {
    // 所有可用的颜色
    colors: colors1,
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
        main: colors1.background2,
        mainLight: colors1.background1,
        mainStrong: colors1.background3,
        primary: colors1.brand1,
        primaryLight: colors.blue4,
        primaryStrong: colors.blue7,
        success: colors.green4,
        successLight: colors.green3,
        successStrong: colors.green7,
        warn: colors.orange4,
        warnLight: colors.orange3,
        warnStrong: colors.orange7,
        error: colors.red4,
        errorLight: colors.red3,
        errorStrong: colors.red7,
        mask: 'rgba(0, 0, 0, 0.7)',
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
            up: `rgba(0, 0, 0, 0.16) 0 -1px 2px -2px,
                rgba(0, 0, 0, 0.12) 0 -3px 6px 0px,
                rgba(0, 0, 0, 0.09) 0px -5px 12px 4px`,
            down: `rgba(0, 0, 0, 0.16) 0 1px 2px -2px,
                rgba(0, 0, 0, 0.12) 0 3px 6px 0pxx,
                rgba(0, 0, 0, 0.09) 0px 5px 12px 4px`,
            left: `rgba(0, 0, 0, 0.16) -1px 0 2px -2px,
                rgba(0, 0, 0, 0.12) -3px 0 6px 0px,
                rgba(0, 0, 0, 0.09) -5px 0 12px 4px`,
            right: `rgba(0, 0, 0, 0.16) 1px 0 2px -2px,
                rgba(0, 0, 0, 0.12) 3px 0 6px 0px,
                rgba(0, 0, 0, 0.09) 5px 0 12px 4px`,
        },
        level2: {
            up: `rgba(0, 0, 0, 0.12) 0 -3px 6px -4px,
                rgba(0, 0, 0, 0.08) 0 -6px 16px 0px,
                rgba(0, 0, 0, 0.05) 0px -9px 28px 8px`,
            down: `rgba(0, 0, 0, 0.12) 0 3px 6px -4px,
                rgba(0, 0, 0, 0.08) 0 6px 16px 0px,
                rgba(0, 0, 0, 0.05) 0px 9px 28px 8px`,
            left: `rgba(0, 0, 0, 0.12) -3px 0 6px -4px,
                rgba(0, 0, 0, 0.08) -6px 0 16px 0px,
                rgba(0, 0, 0, 0.05) -9px 0 28px 8px`,
            right: `rgba(0, 0, 0, 0.12) 3px 0 6px -4px,
                rgba(0, 0, 0, 0.08) 6px 0 16px 0px,
                rgba(0, 0, 0, 0.05) 9px 0 28px 8px`,
        },
        level3: {
            up: `rgba(0, 0, 0, 0.08) 0 -6px 16px -8px,
                rgba(0, 0, 0, 0.05) 0 -9px 28px 0px,
                rgba(0, 0, 0, 0.03) 0px -12px 48px 16px`,
            down: `rgba(0, 0, 0, 0.08) 0 6px 16px -8px,
                rgba(0, 0, 0, 0.05) 0 9px 28px 0px,
                rgba(0, 0, 0, 0.03) 0px 12px 48px 16px`,
            left: `rgba(0, 0, 0, 0.08) -6px 0 16px -8px,
                rgba(0, 0, 0, 0.05) -9px 0 28px 0px,
                rgba(0, 0, 0, 0.03) -12px 0 48px 16px`,
            right: `rgba(0, 0, 0, 0.08) 6px 0 16px -8px,
                rgba(0, 0, 0, 0.05) 9px 0 28px 0px,
                rgba(0, 0, 0, 0.03) 12px 0 48px 16px`,
        },
    },
    // 分割线
    divider: {
        color: {
            regular: colors1.divider1,
            light: colors1.divider2,
            strong: colors1.divider3,
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
        borderColor: {
            ghost: 'transparent',
            main: colors.grey4,
            light: colors.grey2,
            strong: colors.grey6,
            primary: colors.blueBase,
            reactive: {
                mainHover: colors.blue4,
                mainActive: colors.blue6,
                primaryHover: colors.blue4,
                primaryActive: colors.blue4,
            },
        },
    },
    font,
    // 排版布局
    typography: {
        // 标题
        title: genereteTypography(font.size.large, font.weight.medium),
        // 字号比标题大一级
        title1: genereteTypography(font.size.huge, font.weight.medium),
        // 正文/内容
        content: genereteTypography(font.size.regular, font.weight.regular),
        // 辅助信息
        sideText: genereteTypography(font.size.sm, font.weight.regular),
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
};

export default defaultTokens;
