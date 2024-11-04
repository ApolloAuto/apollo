import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'mosaic-custom-toolbar-root': {
        cursor: 'pointer',
        backgroundColor: theme.tokens.backgroundColor.main,
        fontSize: theme.tokens.font.size.large,
        minHeight: '40px',
        maxHeight: '40px',
        lineHeight: theme.tokens.font.lineHeight.regular,
        display: 'flex',
        alignItems: 'center',
        flexDirection: 'row-reverse',
        padding: `0px ${theme.tokens.padding.speace2}`,
        position: 'relative',
    },
    'mosaic-custom-toolbar-title': {
        display: 'flex',
        alignItems: 'center',
        flex: 1,
        ...theme.tokens.typography.title,
        whiteSpace: 'nowrap',
        overflow: 'hidden',
        flexWrap: 'nowrap',
    },
    'mosaic-custom-toolbar-operate': {
        display: 'flex',
    },
    'mosaic-custom-toolbar-operate-item': {
        marginLeft: '10px',
        svg: {
            cursor: 'pointer',
        },
        '&:hover': {
            color: theme.tokens.font.reactive.mainHover,
        },
        '&:active': {
            color: theme.tokens.font.reactive.mainActive,
        },
    },
    'mosaic-custom-toolbar-operate-popover': {
        margin: '-6px 0',
    },
    'mosaic-custom-toolbar-operate-popover-select': {
        cursor: 'pointer',
        padding: '0 12px',
        height: '32px',
        display: 'flex',
        alignItems: 'center',
        ...theme.tokens.typography.content,
        svg: {
            fontSize: theme.tokens.font.size.large,
            marginRight: theme.tokens.margin.speace,
        },
        '&:last-of-type': {
            marginBottom: 0,
        },
        '&:hover': {
            backgroundColor: theme.components.operatePopover.hoverColor,
        },
    },
    'mosaic-custom-toolbar-operate-popover-select-remove': {
        color: theme.tokens.font.color.error,
    },
    'mosaic-custom-toolbar-exit-fullscreen': {
        cursor: 'pointer',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '14px',
        color: '#A6B5CC',
        '&:hover': {
            color: '#fff',
        },
    },
    'mosaic-custom-toolbar-icmove': {
        '& .dreamview-popover-inner-content': {
            ...theme.tokens.typography.content,
            width: '170px',
        },
    },
    'panel-desc-item': {
        lineHeight: '76px',
        height: '76px',

        display: 'flex',
        justifyContent: 'center',
        '&:not(:last-of-type)': {
            borderBottom: '1px solid #383B45',
        },
    },
    'panel-desc-item-left': {
        width: '170px',
    },
    'panel-desc-item-right': {
        width: '584px',
        height: '22px',
        color: theme.tokens.colors.fontColor5,
        fontWeight: 400,
        fontFamily: 'PingFangSC-Regular',
    },
    'btn-item': {
        display: 'inline-block',
        cursor: 'pointer',
        textAlign: 'center',
        minWidth: '32px',
        height: '32px',
        background: theme.components.panelBase.functionRectBgColor,
        borderRadius: '6px',
        marginTop: '12px',
        fontSize: '16px',
        color: theme.components.panelBase.functionRectColor,
        padding: '0 6px 0 6px',
        marginRight: '6px',
        ...theme.tokens.typography.content,
        lineHeight: '32px',
    },
}));
