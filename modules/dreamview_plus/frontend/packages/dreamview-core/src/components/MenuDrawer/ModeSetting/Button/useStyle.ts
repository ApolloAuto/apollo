import { makeStylesWithProps } from '@dreamview/dreamview-theme';

export default makeStylesWithProps<{
    width: number;
}>()((theme, props) => ({
    'mode-setting-button': {
        outline: 'none',
        border: 'none',
        margin: '0',
        width: props.width ? `${props.width}px` : 'auto',
        height: '32px',
        lineHeight: '32px',
        textAlign: 'center',
        borderRadius: '6px',
        padding: '0 20px',
        cursor: 'pointer',
        background: theme.tokens.colors.background2,
        ...theme.tokens.typography.sideText,
        color: theme.tokens.colors.fontColor5,
        '&:hover': {
            color: theme.tokens.colors.fontColor5,
            background: '#353946',
        },
        '&:active': {
            color: theme.tokens.colors.fontColor5,
            background: '#252830',
        },
    },
}));
