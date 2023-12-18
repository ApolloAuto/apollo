import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'dreamview-nodata-placeholder': {
            display: 'flex',
            flexDirection: 'column',
            justifyContent: 'center',
            alignItems: 'center',
            height: '100%',
            width: '100%',
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            color: '#5D6573',
            backgroundColor: '#0F1014',
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
