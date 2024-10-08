import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dreamview-nodata-placeholder': {
        display: 'flex',
        flexDirection: 'column',
        justifyContent: 'center',
        alignItems: 'center',
        height: '100%',
        width: '100%',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '14px',
        color: theme.tokens.colors.fontColor4,
        backgroundColor: theme.tokens.colors.background3,
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
