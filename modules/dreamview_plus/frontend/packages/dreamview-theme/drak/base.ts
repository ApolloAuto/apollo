export const colors = {
    // 品牌色
    brand1: '#1252C0',
    brand2: '#1971E6',
    brand3: '#3288FA',
    brand4: '#579FF1',
    brand5: 'rgba(50,136,250,0.25)',
    brandTransparent: 'rgba(50,136,250,0.25)',

    // 辅助色-错误
    error1: '#CB2B40',
    error2: '#F75660',
    error3: '#F97A7E',
    error4: 'rgba(255,77,88,0.25)',
    errorTransparent: 'rgba(255,77,88,0.25)',

    // 辅助色-警告
    warn1: '#D25F13',
    warn2: '#FF8D26',
    warn3: '#FFAB57',
    warn4: 'rgba(255,141,38,0.25)',
    warnTransparent: 'rgba(255,141,38,0.25)',

    // 辅助色-成功
    success1: '#20A335',
    success2: '#1FCC4D',
    success3: '#69D971',
    success4: 'rgba(31,204,77,0.25)',
    successTransparent: 'rgba(31,204,77,0.25)',

    // 辅助色-黄色
    yellow1: '#C7A218',
    yellow2: '#F3D631',
    yellow3: '#F6E55D',
    yellow4: 'rgba(243,214,49,0.25)',
    yellowTransparent: 'rgba(243,214,49,0.25)',

    // 透明色
    transparent: 'transparent',
    transparent1: 'rgba(115,193,250,0.08)',
    transparent2: 'rgba(0,0,0,0.65)',
    transparent3: 'rgba(80,88,102,0.8)',

    // 背景色
    backgroundMask: 'rgba(255,255,255,0.65)',
    backgroundHover: 'rgba(115,193,250,0.08)',
    background1: '#1A1D24',
    background2: '#343C4D',
    background3: '#0F1014',

    // 字阶色
    fontColor1: '#717A8C',
    fontColor2: '#4D505A',
    fontColor3: '#717A8C',
    fontColor4: '#808B9D',
    fontColor5: '#FFFFFF',
    fontColor6: '#A6B5CC',

    // 分割线
    divider1: '#383C4D',
    divider2: '#383B45',
    divider3: '#252833',
};

export const font = {
    iconReactive: {
        main: colors.fontColor1,
        hover: colors.fontColor3,
        active: colors.fontColor4,
        mainDisabled: '#8c8c8c',
    },
    reactive: {
        mainHover: colors.fontColor5,
        mainActive: '#5D6573',
        mainDisabled: '#40454D',
    },
    color: {
        primary: colors.brand3,
        success: colors.success2,
        warn: colors.warn2,
        error: colors.error2,
        black: colors.fontColor5,
        white: 'white',
        main: colors.fontColor4,
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
