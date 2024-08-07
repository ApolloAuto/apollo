import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'status-ok': {
        width: '64px',
        height: '32px',
        lineHeight: '32px',
        paddingLeft: '10px',
        fontFamily: 'PingFangSC-Regular',
        color: '#1FCC4D',
        fontSize: '14px',
        background: 'rgba(31,204,77,0.10)',
        borderRadius: '6px',
        marginRight: '22px',
    },
    'status-fatal': {
        width: '86px',
        height: '32px',
        lineHeight: '32px',
        paddingLeft: '10px',
        fontFamily: 'PingFangSC-Regular',
        color: '#F75660',
        fontSize: '14px',
        background: 'rgba(247,86,96,0.10)',
        borderRadius: '6px',
    },
    'status-warn': {
        width: '86px',
        height: '32px',
        lineHeight: '32px',
        paddingLeft: '10px',
        fontFamily: 'PingFangSC-Regular',
        color: '#FF8D26',
        fontSize: '14px',
        background: 'rgba(255,141,38,0.10)',
        borderRadius: '6px',
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
