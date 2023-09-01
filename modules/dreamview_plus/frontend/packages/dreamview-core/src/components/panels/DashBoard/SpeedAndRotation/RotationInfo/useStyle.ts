import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'dashboard-rotation-info': {
            display: 'flex',
            flex: '1',
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            background: '#282B36',
            borderRadius: '6px',
            fontFamily: 'PingFangSC-Semibold',
            fontSize: '14px',
            color: '#A6B5CC',
            fontWeight: 600,
            width: '100%',
            height: '100%',
            paddingBottom: '16px',
        },
        'dashboard-rotation-info-font': {
            height: '35px',
            fontFamily: 'DINAlternate-Bold',
            fontSize: '30px',
            color: '#A6B5CC',
            textAlign: 'center',
        },
        'dashboard-rotation-info-font-small': {
            height: '18px',
            fontFamily: 'PingFangSC-Regular',
            fontSize: '12px',
            color: '#808B9D',
        },
        'dashboard-rotation-info-wheel': {
            display: 'flex',
            alignItems: 'center',
            height: '60px',
        },
        error: {
            color: theme.tokens.colors.error,
        },
        info: {
            color: theme.tokens.colors.brand2,
        },
        warn: {
            color: theme.tokens.colors.warn,
        },
    }));
    return hoc();
}
