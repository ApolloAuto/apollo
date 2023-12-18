import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme, prop) => ({
        'source-list-container': {
            '& .source-list-container-item-disbale': {
                cursor: 'not-allowed',
            },
        },
        'source-list-container-item': {
            height: '40px',
            lineHeight: '40px',
            margin: '0 -14px',
            borderRadius: '8px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            cursor: 'pointer',
            '&:hover': {
                background: 'rgba(115,193,250,0.08)',
                '& .source-list-operate-hover': {
                    display: 'block',
                },
            },
        },
        active: {
            backgroundColor: theme.tokens.colors.brand3,
            '& .source-list-operate-hover': {
                display: 'block',
                color: 'white',
            },
            '&:hover': {
                backgroundColor: theme.tokens.colors.brand3,
            },
            '& > span': {
                color: theme.tokens.colors.fontColor1,
            },
        },
        'source-list-name': {
            ...theme.util.textEllipsis,
            ...theme.tokens.typography.content,
            width: '250px',
            marginLeft: '36px',
            whiteSpace: 'nowrap',
        },
        'source-list-operate': {
            marginRight: '14px',
            display: 'none',
            fontSize: theme.tokens.font.size.large,
        },
        'source-list-title': {
            height: '40px',
            display: 'flex',
            alignItems: 'center',
            '&:hover .source-list-title-text-hover': {
                color: theme.tokens.colors.brand2,
            },
        },
        'source-list-title-icon-expand': {
            transform: 'rotateZ(0)',
        },
        'source-list-title-icon': {
            fontSize: theme.tokens.font.size.large,
            marginRight: '6px',
            transition: theme.tokens.transitions.easeInOut(),
            transform: 'rotateZ(-90deg)',
        },
        'source-list-title-text': {
            cursor: 'pointer',
            width: '250px',
            ...theme.util.textEllipsis,
            whiteSpace: 'nowrap',
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
            color: theme.tokens.colors.fontColor3,
            img: {
                display: 'block',
                margin: '0 auto',
            },
        },
        'empty-msg': {
            '& > span': {
                color: theme.tokens.colors.brand2,
                cursor: 'pointer',
            },
        },
    }));
    return hoc;
}
