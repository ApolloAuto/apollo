import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'panel-dash-board': {
            display: 'flex',
            flexDirection: 'column',
            rowGap: '8px',
            height: '100%',
            width: '100%',
            minWidth: '380px',
            padding: '16px',
            background: theme.components.dashBoard.bgColor,
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
