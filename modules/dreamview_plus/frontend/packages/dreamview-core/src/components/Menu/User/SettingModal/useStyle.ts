import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'setting-modal': {
            '&.dreamview-modal-root .dreamview-modal .dreamview-modal-content': {
                padding: '20px',
            },
            '&.dreamview-modal-root .dreamview-modal .dreamview-modal-header': {
                margin: '-20px',
                marginBottom: '16px',
                padding: '0 20px',
            },
            '&.dreamview-modal-root .dreamview-modal .dreamview-modal-footer': {
                marginTop: '20px',
            },
        },
        'setting-root': {
            display: 'flex',
            marginTop: theme.tokens.margin.speace2,
        },
        'setting-right': {
            flex: 1,
            marginLeft: '12px',
            background: theme.components.settingModal.cardBgColor,
            borderRadius: '8px',
            height: '358px',
            padding: theme.tokens.padding.speace2,
        },
        tabs: {
            width: '144px',
            borderRight: `1px solid ${theme.tokens.colors.divider2}`,
            height: '358px',
        },
        'tabs-item': {
            ...theme.tokens.typography.content,
            width: '132px',
            height: '32px',
            lineHeight: '32px',
            borderRadius: '6px',
            color: theme.components.settingModal.tabColor,
            cursor: 'pointer',
            textIndent: '16px',
            marginBottom: '6px',
            '&:hover': {
                background: theme.components.settingModal.tabBgHoverColor,
            },
        },
        active: {
            color: theme.components.settingModal.tabActiveColor,
            backgroundColor: theme.components.settingModal.tabActiveBgColor,
            '&:hover': {
                backgroundColor: theme.components.settingModal.tabActiveBgColor,
            },
        },
        laguage: {
            '& > p': {
                ...theme.tokens.typography.title,
                color: theme.tokens.colors.fontColor6,
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
            height: '90px',
            marginLeft: '-18px',
            display: 'block',
            marginTop: '-34px',
            marginBottom: '-18px',
        },
        about: {
            ...theme.tokens.typography.content,
            color: theme.tokens.colors.fontColor4,
        },
        aboutitem: {
            marginBottom: theme.tokens.margin.speace,
        },
        blod: {
            fontWeight: 500,
            color: theme.tokens.colors.fontColor5,
            marginBottom: '6px',
        },
        divider: {
            height: '1px',
            background: theme.tokens.colors.divider2,
            margin: `${theme.tokens.margin.speace2} 0`,
        },
    }));

    return hoc();
}
