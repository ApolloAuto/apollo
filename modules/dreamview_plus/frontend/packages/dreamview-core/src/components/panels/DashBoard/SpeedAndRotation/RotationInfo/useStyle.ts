import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dashboard-rotation-info': {
        display: 'flex',
        flex: '1',
        flexDirection: 'column',
        justifyContent: 'center',
        alignItems: 'center',
        background: theme.components.dashBoard.cardBgColor,
        borderRadius: '6px',
        fontFamily: 'PingFangSC-Semibold',
        fontSize: '14px',
        color: theme.components.dashBoard.lightFontColor,
        fontWeight: 600,
        width: '100%',
        height: '100%',
        paddingBottom: '16px',
    },
    'dashboard-rotation-info-font': {
        height: '35px',
        fontFamily: 'DINAlternate-Bold',
        fontSize: '30px',
        color: theme.components.dashBoard.color,
        textAlign: 'center',
    },
    'dashboard-rotation-info-font-small': {
        height: '18px',
        fontFamily: 'PingFangSC-Regular',
        fontSize: '12px',
    },
    'dashboard-rotation-info-wheel': {
        display: 'flex',
        alignItems: 'center',
        height: '60px',
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
