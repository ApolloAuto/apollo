import { makeStylesWithProps } from '@dreamview/dreamview-theme';

export default makeStylesWithProps<{
    height?: number;
}>()((theme, prop) => ({
    'source-list-container': {
        '& .source-list-container-item-disbale': {
            cursor: 'not-allowed',
        },
    },
    'source-list-container-item': {
        height: '32px',
        lineHeight: '32px',
        borderRadius: '8px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        cursor: 'pointer',
        color: theme.components.sourceItem.color,
        padding: '0 12px',
        '&:hover': {
            background: theme.components.sourceItem.bgColorHover,
            '& .source-list-operate-hover': {
                display: 'block',
            },
        },
    },
    active: {
        backgroundColor: theme.components.sourceItem.activeBgColor,
        '& .source-list-operate-hover': {
            display: 'block',
            color: theme.components.sourceItem.activeIconColor,
        },
        '&:hover': {
            backgroundColor: theme.components.sourceItem.activeBgColor,
        },
        '& > span': {
            color: theme.components.sourceItem.activeColor,
        },
    },
    'source-list-name': {
        ...theme.util.textEllipsis,
        ...theme.tokens.typography.content,
        lineHeight: '32px',
        width: '250px',
        whiteSpace: 'nowrap',
    },
    'source-list-operate': {
        display: 'none',
        fontSize: theme.tokens.font.size.large,
    },
    'source-list-title': {
        height: '40px',
        display: 'flex',
        alignItems: 'center',
    },
    'source-list-title-icon-expand': {
        transform: 'rotateZ(0)',
    },
    'source-list-title-icon': {
        fontSize: theme.tokens.font.size.large,
        color: theme.tokens.colors.fontColor6,
        marginRight: '6px',
        transition: theme.tokens.transitions.easeInOut(),
        transform: 'rotateZ(-90deg)',
    },
    'source-list-title-text': {
        cursor: 'pointer',
        width: '250px',
        ...theme.util.textEllipsis,
        whiteSpace: 'nowrap',
        color: theme.tokens.colors.fontColor6,
        '&:hover': {
            color: theme.tokens.font.reactive.mainHover,
        },
    },
    'source-list-close': {
        height: 0,
        overflowY: 'hidden',
        transition: theme.tokens.transitions.easeInOut(),
        '& > div': {
            margin: '0 14px',
        },
    },
    'source-list-expand': {
        height: `${prop?.height}px`,
    },
    empty: {
        textAlign: 'center',
        color: theme.tokens.colors.fontColor4,
        img: {
            display: 'block',
            margin: '0 auto',
            width: '160px',
        },
    },
    'empty-msg': {
        '& > span': {
            color: theme.tokens.colors.brand3,
            cursor: 'pointer',
        },
    },
}));
