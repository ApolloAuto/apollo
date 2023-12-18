import { useMakeStyle } from '@dreamview/dreamview-theme';
import tinycolor from 'tinycolor2';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'setting-root': {
            display: 'flex',
            marginTop: theme.tokens.margin.speace3,
        },
        'setting-right': {
            flex: 1,
            marginLeft: '12px',
            background: '#181A1F',
            borderRadius: '8px',
            height: '358px',
            padding: theme.tokens.padding.speace3,
        },
        tabs: {
            width: '144px',
            borderRight: '1px solid #383B45',
            height: '358px',
        },
        'tabs-item': {
            ...theme.tokens.typography.content,
            width: '132px',
            height: '32px',
            lineHeight: '32px',
            borderRadius: '6px',
            color: theme.tokens.colors.fontColor3,
            cursor: 'pointer',
            textIndent: '16px',
            marginBottom: '6px',
            '&:hover': {
                backgroundColor: tinycolor(theme.tokens.colors.background2).setAlpha(0.5).toRgbString(),
            },
        },
        active: {
            color: theme.tokens.colors.fontColor1,
            backgroundColor: theme.tokens.colors.brand2,
            '&:hover': {
                backgroundColor: theme.tokens.colors.brand2,
            },
        },
        laguage: {
            '& > p': {
                ...theme.tokens.typography.content,
                color: theme.tokens.colors.fontColor2,
                marginBottom: theme.tokens.margin.speace,
            },
        },
        checkboxitem: {
            display: 'flex',
            alignItems: 'center',
        },
        checkbox: {
            height: '16px',
            marginRight: theme.tokens.margin.speace,
            '.rc-checkbox-input': {
                width: '16px',
                height: '16px',
            },
            '&:not(.rc-checkbox-checked) .rc-checkbox-input': {
                background: 'transparent',
            },
        },
        logo: {
            height: 46,
            marginLeft: -6,
        },
        about: {
            ...theme.tokens.typography.content,
            color: theme.tokens.colors.fontColor3,
        },
        aboutitem: {
            marginBottom: theme.tokens.margin.speace,
        },
        blod: {
            fontWeight: 500,
            color: theme.tokens.colors.fontColor2,
            marginBottom: '6px',
        },
        divider: {
            height: '1px',
            background: '#383b45',
            margin: `${theme.tokens.margin.speace2} 0`,
        },
    }));

    return hoc();
}
