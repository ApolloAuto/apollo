import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'flex-center': {
        display: 'flex',
    },
    disabled: {
        color: '#40454D',
        '& .anticon': {
            color: '#383d47',
            cursor: 'not-allowed',
        },
        '& .progress-pointer': {
            display: 'none',
        },
    },
    'record-controlbar-container': {
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        padding: `0 ${theme.tokens.padding.speace3}`,
        color: theme.tokens.colors.fontColor4,
        '& .ic-play-container': {
            height: '40px',
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
        },
        '& .anticon': {
            fontSize: theme.tokens.font.size.large,
            color: theme.tokens.colors.fontColor5,
        },
        '& .ic-play-btn': {
            cursor: 'pointer',
            fontSize: '24px !important',
            marginRight: theme.tokens.margin.speace2,
        },
        '& .ic-stop-btn': {
            cursor: 'pointer',
            fontSize: '22px',
            marginRight: '6px',
            color: '#F75660',
            '&:active': {
                color: 'rgba(247,56,96,0.8)',
            },
        },
        '& .record-start-record-btn': {
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
        '& .record-download-btn': {
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
        '& .record-download-btn-text': {
            fontSize: theme.tokens.font.size.sm,
        },
        '& .record-reset-btn': {
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
        '& .record-download-reset-text': {
            fontSize: theme.tokens.font.size.sm,
        },
    },
    'operate-success': {
        '& .dreamview-popover-inner,& .dreamview-popover-arrow::before, & .dreamview-popover-arrow::after': {
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
