import { useMakeStyle } from '@dreamview/dreamview-theme';
import tinycolor from 'tinycolor2';

export default function useStyle() {
    const hoc = useMakeStyle((theme, props) => ({
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
            background: theme.tokens.colors.background1,
            ...theme.tokens.typography.sideText,
            color: theme.tokens.colors.fontColor2,
            '&:hover': {
                color: theme.tokens.colors.fontColor1,
                background: '#353946',
            },
            '&:active': {
                color: theme.tokens.colors.fontColor2,
                background: '#252830',
            },
        },
    }));
    return hoc;
}
