import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    disabled: {
        color: '#40454D',
        '& .anticon': {
            color: '#303339',
            cursor: 'not-allowed',
        },
        '& .progress-pointer': {
            display: 'none',
        },
    },
    'player-controlbar-container': {
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        padding: `0 ${theme.tokens.padding.speace3}`,
        color: theme.tokens.colors.fontColor4,
        '& .anticon': {
            fontSize: theme.tokens.font.size.large,
            color: theme.tokens.colors.fontColor5,
        },
        '& .ic-play-btn': {
            cursor: 'pointer',
            fontSize: '24px !important',
            marginRight: '6px',
        },
        '& .player-progress-text': {
            marginRight: theme.tokens.margin.speace3,
            fontSize: theme.tokens.font.size.regular,
            width: 110,
        },
        '& .player-progress': {
            flex: 1,
            marginRight: theme.tokens.margin.speace2,
        },
        '& .player-download-btn': {
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            flexDirection: 'column',
            marginRight: '28px',
            '&:hover': {
                color: theme.tokens.font.reactive.mainHover,
                '& .anticon': {
                    color: theme.tokens.font.reactive.mainHover,
                },
            },
            '&:active': {
                color: theme.tokens.font.reactive.mainActive,
                '& .anticon': {
                    color: theme.tokens.font.reactive.mainActive,
                },
            },
        },
        '& .player-download-btn-text': {
            fontSize: theme.tokens.font.size.sm,
        },
        '& .player-reset-btn': {
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            flexDirection: 'column',
            '&:hover': {
                color: theme.tokens.font.reactive.mainHover,
                '& .anticon': {
                    color: theme.tokens.font.reactive.mainHover,
                },
            },
            '&:active': {
                color: theme.tokens.font.reactive.mainActive,
                '& .anticon': {
                    color: theme.tokens.font.reactive.mainActive,
                },
            },
        },
        '& .player-download-reset-text': {
            fontSize: theme.tokens.font.size.sm,
        },
    },
    'operate-success': {
        '& .dreamview-popover-inner, & .dreamview-popover-arrow::after': {
            background: 'rgba(31,204,77,0.25)',
        },
        '& .dreamview-popover-arrow::before': {
            background: 'rgba(31,204,77,0.25)',
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(31,204,77,0.25)',
        },
        '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
            color: theme.tokens.colors.success2,
        },
    },
    'operate-failed': {
        '& .dreamview-popover-inner, & .dreamview-popover-arrow::after': {
            background: 'rgba(255,77,88,0.25)',
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(255,77,88,0.25)',
        },
        '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
            color: '#FF4D58',
        },
    },
}));
