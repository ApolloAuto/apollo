import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dashboard-signal': {
        display: 'flex',
        flex: '1',
        justifyContent: 'center',
        alignItems: 'center',
        background: theme.components.dashBoard.cardBgColor,
        borderRadius: '6px',
        fontFamily: 'PingFangSC-Semibold',
        fontSize: '14px',
        color: theme.components.dashBoard.color,
        fontWeight: 600,
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
