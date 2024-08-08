import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'mode-setting-title': {
        ...theme.tokens.typography.title,
        paddingLeft: '10px',
        position: 'relative',
        cursor: 'pointer',
        marginBottom: '12px',
        '&::after': {
            position: 'absolute',
            width: '2px',
            height: '12px',
            background: theme.tokens.colors.brand3,
            content: '""',
            left: 0,
            top: 0,
            bottom: 0,
            margin: 'auto 0',
        },
        '& .anticon': {
            position: 'absolute',
            right: 0,
            top: '4px',
            transform: 'rotate(0)',
            color: theme.tokens.colors.fontColor1,
            transition: theme.tokens.transitions.easeInOut(),
        },
    },
    'mode-setting-icon-active': {
        '& .anticon': {
            transform: 'rotate(180deg)',
        },
    },
    'mode-setting-expend': {
        transition: theme.tokens.transitions.easeInOut(),
        height: 'auto',
        overflow: 'hidden',
        padding: '0 14px',
        margin: '0 -14px',
    },
    'overflow-hidden': {
        overflow: 'hidden',
    },
}));
