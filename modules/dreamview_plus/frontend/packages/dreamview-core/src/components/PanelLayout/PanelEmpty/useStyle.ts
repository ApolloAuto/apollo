import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'panel-item-add': {
        position: 'absolute',
        left: 0,
        top: 0,
        right: 0,
        bottom: 0,
        display: 'none',
        alignItems: 'center',
        flexDirection: 'column',
        justifyContent: 'center',
        background: theme.components.addPanel.maskColor,
        borderRadius: '8px',
        color: theme.tokens.colors.fontColor5,
        fontSize: theme.tokens.font.size.regular,
        '& .anticon': {
            fontSize: '22.5px',
            marginBottom: theme.tokens.margin.speace,
        },
    },
    'panel-item-title': {
        ...theme.tokens.typography.content,
        ...theme.util.textEllipsis,
        color: theme.components.addPanel.titleColor,
        marginBottom: '2px',
    },
    'panel-item-desc': {
        ...theme.tokens.typography.sideText,
        ...theme.util.textEllipsis2,
        color: theme.components.addPanel.contentColor,
    },
    'panel-item-img': {
        width: '100%',
        paddingBottom: '75%',
        background: theme.components.addPanel.coverImgBgColor,
        marginBottom: '6px',
        backgroundSize: '100%',
    },
    'panel-item-add-icon': {
        width: '32px',
        height: '32px',
        borderRadius: '32px',
        overflow: 'hidden',
        marginBottom: '8px',
        boxShadow: `0 0 16px 0 ${theme.tokens.colors.brand2}`,
        '& img': {
            width: '44px',
            marginLeft: '-6px',
            marginTop: '-6px',
        },
    },
    'panel-empty-container': {
        width: '100%',
        height: '100%',
        background: theme.tokens.colors.background3,
        display: 'flex',
        flexDirection: 'column',
    },
    'panel-item': {
        cursor: 'pointer',
        position: 'relative',
        padding: '12px',
        background: theme.components.addPanel.bgColor,
        boxShadow: theme.components.addPanel.boxShadow,
        border: theme.components.addPanel.border,
        borderRadius: '8px',
        marginBottom: theme.tokens.margin.speace,
        '&:hover .panel-item-add-hover': {
            display: 'flex',
        },
        '&:hover': {
            boxShadow: theme.components.addPanel.boxShadowHover,
        },
    },
    'panel-empty-title': {
        lineHeight: '78px',
        padding: `0 ${theme.tokens.margin.speace3}`,
        fontFamily: 'PingFangSC-Semibold',
        fontSize: '20px',
        fontWeight: 600,
        color: theme.components.addPanel.titleColor,
    },
    'panel-list-container': {
        flex: 1,
    },
    'panel-list': {
        padding: `0 ${theme.tokens.margin.speace3}`,
        display: 'flex',
        flexWrap: 'wrap',
        justifyContent: 'space-between',
    },
}));
