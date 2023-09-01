import baseToken, { ISize } from '../baseToken';

type IButtonType = 'primary' | 'text' | 'dashed';

interface IGenerateButtonTheme {
    type: IButtonType;
    size: ISize;
    ghost: boolean;
}

const buttonUtil = {
    fontSize(size: ISize) {
        return baseToken.font.size[size];
    },
    height(size: ISize) {
        const h = (val: number) => ({
            height: `${val}px`,
            lineHeight: `${val}px`,
        });
        return {
            sm: h(22),
            regular: h(32),
            large: h(40),
            huge: h(40),
        }[size];
    },
    padding(size: ISize) {
        return {
            sm: '0 12px',
            regular: '0 18px',
            large: '0 22px',
            huge: '0 22px',
        }[size];
    },
    common(width: number | string) {
        return {
            width,
            whiteSpace: 'nowrap',
            boxSizing: 'border-box',
            outline: 'none',
            position: 'relative',
            display: 'inline-flex',
            justifyContent: 'center',
            alignItems: 'center',
            userSelect: 'none',
            verticalAlign: 'middle',
            transitionProperty: 'border-color, background-color, color, opacity, box-shadow',
            transitionDuration: '.2s',
        };
    },
    border(type: IButtonType, ghost: boolean) {
        if (type === 'text') {
            return {
                borderWidth: baseToken.border.width,
                borderColor: baseToken.border.borderColor.transparent,
                borderStyle: 'solid',
            };
        }
        if (type === 'dashed') {
            return {
                borderWidth: baseToken.border.width,
                borderColor: baseToken.border.borderColor.main,
                borderStyle: 'dashed',
            };
        }
        if (ghost) {
            return {
                borderWidth: baseToken.border.width,
                borderColor: baseToken.border.borderColor.main,
                borderStyle: 'solid',
            };
        }
        return {
            borderWidth: baseToken.border.width,
            borderColor: baseToken.border.borderColor.primary,
            borderStyle: 'solid',
        };
    },
    fontColor(type: IButtonType, ghost: boolean) {
        if (type === 'text') {
            return baseToken.font.color.black;
        }
        if (ghost) {
            return baseToken.font.color.black;
        }
        return baseToken.font.color.main;
    },
    backgroundColor(type: IButtonType, ghost: boolean) {
        if (type === 'text') {
            return baseToken.backgroundColor.transparent;
        }
        if (ghost) {
            return baseToken.backgroundColor.transparent;
        }
        return baseToken.backgroundColor.primary;
    },
    reactive(type: IButtonType, ghost: boolean) {
        return {
            '&:not(:disabled):active': {},
            '&:not(:disabled):hover': {},
            '&:disabled': {},
        };
    },
};

function generateButtonTheme(props: IGenerateButtonTheme) {
    return {
        ...buttonUtil.common('auto'),
        ...buttonUtil.height(props.size),
        ...buttonUtil.border(props.type, props.ghost),
        ...buttonUtil.reactive(props.type, props.ghost),
        padding: buttonUtil.padding(props.size),
        color: buttonUtil.fontColor(props.type, props.ghost),
        backgroundColor: buttonUtil.backgroundColor(props.type, props.ghost),
        fontSize: buttonUtil.fontSize(props.size),
    };
}
// type: IButtonType;
// size: ISize;
// ghost: boolean;
// minWidth: number;
export default {
    primary: generateButtonTheme({ type: 'primary', size: ISize.regular, ghost: false }),
    primaryGhost: generateButtonTheme({ type: 'primary', size: ISize.regular, ghost: true }),
    text: generateButtonTheme({ type: 'text', size: ISize.regular, ghost: false }),
    dashed: generateButtonTheme({ type: 'dashed', size: ISize.regular, ghost: false }),
    primarySm: generateButtonTheme({ type: 'primary', size: ISize.sm, ghost: false }),
    primaryGhostSm: generateButtonTheme({ type: 'primary', size: ISize.sm, ghost: true }),
    textSm: generateButtonTheme({ type: 'text', size: ISize.sm, ghost: false }),
    dashedSm: generateButtonTheme({ type: 'dashed', size: ISize.sm, ghost: false }),
    primaryLarge: generateButtonTheme({ type: 'primary', size: ISize.large, ghost: false }),
    primaryGhostLarge: generateButtonTheme({ type: 'primary', size: ISize.large, ghost: true }),
    textLarge: generateButtonTheme({ type: 'text', size: ISize.large, ghost: false }),
    dashedLarge: generateButtonTheme({ type: 'dashed', size: ISize.large, ghost: false }),
};
