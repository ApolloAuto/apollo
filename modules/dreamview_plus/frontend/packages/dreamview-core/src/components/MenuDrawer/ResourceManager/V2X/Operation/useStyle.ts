import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'source-operate': {
        display: 'flex',
        '& > span': {
            marginRight: '32px',
            cursor: 'pointer',
            '&:hover': {
                color: theme.tokens.font.reactive.mainHover,
            },
            '&:active': {
                color: theme.tokens.font.reactive.mainActive,
            },
        },
        '& .anticon': {
            display: 'block',
            fontSize: theme.tokens.font.size.large,
        },
    },
    retry: {
        '& .anticon': {
            paddingTop: '1px',
            fontSize: `${theme.tokens.font.size.regular} !important`,
        },
    },
    'source-operate-icon': {
        fontSize: theme.tokens.font.size.large,
        cursor: 'pointer',
        marginRight: '32px',
    },
    disabled: {
        display: 'flex',
        '& > span': {
            cursor: 'not-allowed',
            color: theme.tokens.font.reactive.mainDisabled,
        },
    },
    font18: {
        '& .anticon': {
            fontSize: '18px',
        },
    },
}));
