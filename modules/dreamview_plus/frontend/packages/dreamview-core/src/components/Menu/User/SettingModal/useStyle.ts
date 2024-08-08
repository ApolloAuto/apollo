import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
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
    'bottom-btn': {
        position: 'absolute !important',
        right: 0,
        bottom: '-52px',
    },
    'setting-right': {
        position: 'relative',
        flex: 1,
        marginLeft: '12px',
        background: theme.components.settingModal.cardBgColor,
        borderRadius: '8px',
        height: '373px',
        padding: theme.tokens.padding.speace2,
        marginBottom: '52px',
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
    'device-table': {
        table: {
            width: '100%',
            borderCollapse: 'separate',
            borderSpacing: 0,
        },
        '.rc-table-thead': {
            backgroundColor: '#323642',
            height: '36px',
            fontFamily: 'PingFangSC-Medium',
            fontSize: '14px',
            color: '#A6B5CC',
            whiteSpace: 'nowrap',
            textAlign: 'left',
            th: {
                padding: '0 20px',
                '&:first-of-type': {
                    textIndent: '22px',
                },
            },
        },
        '.rc-table-tbody': {
            td: {
                backgroundColor: '#181A1F',
                padding: '0 20px',
                height: '36px',
                fontFamily: 'PingFangSC-Regular',
                fontSize: '14px',
                color: '#A6B5CC',
                fontWeight: 400,
                borderBottom: '1px solid #292C33',
            },
        },
    },
    'device-product': {
        display: 'flex',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '12px',
        fontWeight: 400,
    },
    'device-tag': {
        color: '#3288FA',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '12px',
        fontWeight: 400,
        padding: '0 4px',
        height: '20px',
        lineHeight: '20px',
        background: 'rgba(50,136,250,0.25)',
        borderRadius: '4px',
        marginRight: '4px',
        '&:last-of-type': {
            marginRight: 0,
        },
    },
    'float-left': {
        float: 'left',
    },
    'device-flex': {
        overflow: 'hidden',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '14px',
        color: '#A6B5CC',
        lineHeight: '22px',
        fontWeight: 400,
        marginBottom: '6px',
        '& > div': {
            float: 'left',
        },
    },
    'device-label': {
        minWidth: '86px',
    },
    'device-value': {
        overflow: 'hidden',
    },
    'not-login': {
        textAlign: 'center',
        img: {
            display: 'block',
            width: '160px',
            height: '100px',
            margin: '67px auto 0',
        },
        p: {
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            color: '#A6B5CC',
            textAlign: 'center',
            fontWeight: '400',
        },
        div: {
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            color: '#808B9D',
            textAlign: 'center',
            fontWeight: 400,
            marginTop: '6px',
        },
    },
    'account-flex': {
        display: 'flex',
        color: '#808B9D',
        marginBottom: '16px',
        '.dreamview-radio-wrapper': {
            color: '#808B9D',
        },
    },
}));
