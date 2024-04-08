import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'dash-board-signal-gear': {
            display: 'flex',
            flexDirection: 'row',
            flex: '1',
            columnGap: '10px',
            height: '100%',
            width: '100%',
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
    return hoc();
}
