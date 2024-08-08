import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'player-download-btn': {
        cursor: 'pointer',
        display: 'flex',
        alignItems: 'center',
        flexDirection: 'column',
        marginRight: '28px',
        '&.disabled': {
            color: '#40454D',
            cursor: 'not-allowed',
            '& .anticon': {
                color: '#383d47',
            },
        },
        '&:hover:not(.disabled)': {
            color: theme.tokens.font.reactive.mainHover,
            '& .anticon': {
                color: theme.tokens.font.reactive.mainHover,
            },
        },
        '&:active:not(.disabled)': {
            color: theme.tokens.font.reactive.mainActive,
            '& .anticon': {
                color: theme.tokens.font.reactive.mainActive,
            },
        },
    },
    'player-download-btn-text': {
        fontSize: theme.tokens.font.size.sm,
    },
    'player-reset-btn': {
        cursor: 'pointer',
        display: 'flex',
        alignItems: 'center',
        flexDirection: 'column',
        '&.disabled': {
            color: '#40454D',
            cursor: 'not-allowed',
            '& .anticon': {
                color: '#383d47',
            },
        },
        '&:hover:not(.disabled)': {
            color: theme.tokens.font.reactive.mainHover,
            '& .anticon': {
                color: theme.tokens.font.reactive.mainHover,
            },
        },
        '&:active:not(.disabled)': {
            color: theme.tokens.font.reactive.mainActive,
            '& .anticon': {
                color: theme.tokens.font.reactive.mainActive,
            },
        },
    },
    'player-download-reset-text': {
        fontSize: theme.tokens.font.size.sm,
    },
    'player-record-btn': {
        cursor: 'pointer',
        display: 'flex',
        alignItems: 'center',
        flexDirection: 'column',
        marginRight: '28px',
        fontSize: theme.tokens.font.size.large,
        color: theme.tokens.colors.fontColor6,
        '&:hover:not(.disabled)': {
            color: theme.tokens.font.reactive.mainHover,
            '& .anticon': {
                color: theme.tokens.font.reactive.mainHover,
            },
        },
        '&:active:not(.disabled)': {
            color: theme.tokens.font.reactive.mainActive,
            '& .anticon': {
                color: theme.tokens.font.reactive.mainActive,
            },
        },
    },
    'player-record-text': {
        fontSize: theme.tokens.font.size.sm,
    },
    'operate-success': {
        '&.dreamview-popover .dreamview-popover-content .dreamview-popover-inner': {
            background: theme.tokens.colors.success4,
        },
        '& .dreamview-popover-arrow::after': {
            background: theme.tokens.colors.success4,
        },
        '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
            color: theme.tokens.colors.success3,
        },
    },
    'operate-failed': {
        '&.dreamview-popover .dreamview-popover-content .dreamview-popover-inner': {
            background: theme.tokens.colors.error4,
        },
        '& .dreamview-popover-arrow::after': {
            background: theme.tokens.colors.error4,
        },
        '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
            color: theme.tokens.colors.error3,
        },
    },
}));
