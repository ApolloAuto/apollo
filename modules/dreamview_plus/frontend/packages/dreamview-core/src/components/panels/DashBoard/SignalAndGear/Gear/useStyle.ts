import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dashboard-gear': {
        display: 'flex',
        flex: '1',
        justifyContent: 'space-between',
        alignItems: 'center',
        columnGap: '15px',
        background: theme.components.dashBoard.cardBgColor,
        borderRadius: '6px',
        fontFamily: 'PingFangSC-Semibold',
        fontSize: '14px',
        color: theme.components.dashBoard.color,
        fontWeight: 600,
        width: '100%',
        height: '100%',
    },
    'dashboard-gear-position': {
        width: '24px',
        height: '24px',
        lineHeight: '24px',
        textAlign: 'center',
    },
    'dashboard-gear-position-active': {
        width: '24px',
        height: '24px',
        borderRadius: '4px',
        background: '#3288FA',
        color: '#fff',
        lineHeight: '24px',
        textAlign: 'center',
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
