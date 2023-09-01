import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'panel-dash-board': {
            display: 'flex',
            flexDirection: 'column',
            rowGap: '8px',
            height: '100%',
            width: '100%',
            padding: '16px',
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
