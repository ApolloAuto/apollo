export const colors = {
    // 品牌色
    brand1: '#044CB9',
    brand2: '#055FE7',
    brand3: '#347EED',
    brand4: '#CFE5FC',
    brand5: '#E6EFFC',
    brandTransparent: 'rgba(50,136,250,0.25)',

    // 辅助色-错误
    error1: '#CC2B36',
    error2: '#F53145',
    error3: '#FF5E69',
    error4: '#FCEDEF',
    errorTransparent: 'rgba(255, 77, 88, 0.25)',

    // 辅助色-警告
    warn1: '#CC5A04',
    warn2: '#FF6F00',
    warn3: '#FF8D37',
    warn4: '#FFF1E5',
    warnTransparent: 'rgba(255,141,38,0.25)',

    // 辅助色-成功
    success1: '#009072',
    success2: '#00B48F',
    success3: '#33C3A5',
    success4: '#DFFBF2',
    successTransparent: 'rgba(31,204,77,0.25)',

    // 辅助色-黄色
    yellow1: '#C79E07',
    yellow2: '#F0C60C',
    yellow3: '#F3D736',
    yellow4: '#FDF9E6',
    yellowTransparent: 'rgba(243,214,49,0.25)',

    // 透明色
    transparent: 'transparent',
    transparent1: '#F5F6F8',
    transparent2: 'rgba(0,0,0,0.45)',
    transparent3: 'rgba(200,201,204,0.6)',

    // 背景色
    backgroundMask: 'rgba(255,255,255,0.65)',
    backgroundHover: 'rgba(115,193,250,0.08)',

    background1: '#FFFFFF',
    background2: '#FFFFFF',
    background3: '#F5F7FA',

    // 字阶色
    fontColor1: '#C8CACD',
    fontColor2: '#C8CACD',
    fontColor3: '#A0A3A7',
    fontColor4: '#6E7277',
    fontColor5: '#232A33',
    fontColor6: '#232A33',

    // 分割线
    divider1: '#DBDDE0',
    divider2: '#DBDDE0',
    divider3: '#EEEEEE',
};

export const font = {
    iconReactive: {
        main: colors.fontColor1,
        hover: colors.fontColor3,
        active: colors.fontColor4,
        mainDisabled: '#8c8c8c',
    },
    reactive: {
        mainHover: colors.brand2,
        mainActive: colors.brand1,
        mainDisabled: '#8c8c8c',
    },
    color: {
        primary: colors.brand3,
        success: colors.success2,
        warn: colors.warn2,
        error: colors.error2,
        black: colors.fontColor5,
        white: 'white',
        main: '#282F3C',
        mainLight: colors.fontColor6,
        mainStrong: colors.fontColor5,

        colorInBrand: 'white',
        colorInBackground: colors.fontColor5,
        colorInBackgroundHover: colors.fontColor5,
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

export default colors;
