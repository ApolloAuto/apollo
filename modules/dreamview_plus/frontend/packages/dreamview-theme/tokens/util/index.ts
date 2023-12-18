import { CSSObject } from 'tss-react';

export default {
    flex(
        direction: 'row' | 'column' = 'row',
        justifyContent: CSSObject['justifyContent'] = 'center',
        alignItems: CSSObject['alignItems'] = 'center',
    ) {
        return {
            display: 'flex',
            flexDirection: direction,
            justifyContent,
            alignItems,
        };
    },
    flexCenterCenter: {
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
    },
    func: {
        textReactive(hoverColor: string, activeColor: string) {
            return {
                '&:hover': {
                    color: hoverColor,
                },
                '&:active': {
                    color: activeColor,
                },
            };
        },
    },
    textEllipsis: {
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        whiteSpace: 'nowrap',
    },
    textEllipsis2: {
        width: '100%',
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        display: '-webkit-box',
        '-WebkitLineClamp': '2',
        '-WebkitBoxOrient': 'vertical',
    },
    scrollX: {
        'overflow-x': 'hidden',
        '&:hover': {
            'overflow-x': 'auto',
        },
    },
    scrollY: {
        'overflow-y': 'hidden',
        '&:hover': {
            'overflow-y': 'auto',
        },
    },
    scroll: {
        overflow: 'hidden',
        '&:hover': {
            overflow: 'auto',
        },
    },
    scrollXI: {
        'overflow-x': 'hidden !important',
        '&:hover': {
            'overflow-x': 'auto !important',
        },
    },
    scrollYI: {
        'overflow-y': 'hidden !important',
        '&:hover': {
            'overflow-y': 'auto !important',
        },
    },
    scrollI: {
        overflow: 'hidden !important',
        '&:hover': {
            overflow: 'auto !important',
        },
    },
};
