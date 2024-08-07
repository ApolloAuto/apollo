import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'cm-container': {
        display: 'flex',
        width: '100%',
        height: '100%',
    },
    error: {
        color: theme.tokens.colors.error2,
    },
    info: {
        color: theme.tokens.colors.brand3,
    },
    warn: {
        color: theme.tokens.colors.warn2,
    },
}));
