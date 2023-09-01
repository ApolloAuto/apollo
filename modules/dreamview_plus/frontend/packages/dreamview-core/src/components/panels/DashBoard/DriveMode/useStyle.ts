import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'dashboard-drive-mode': {
            display: 'flex',
            flex: '1',
            justifyContent: 'center',
            alignItems: 'center',
            background: '#282B36',
            borderRadius: '6px',
            fontFamily: 'PingFangSC-Semibold',
            fontSize: '14px',
            color: '#A6B5CC',
            fontWeight: 600,
            height: '100%',
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
