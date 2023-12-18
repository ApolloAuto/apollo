import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'charts-operation': {
            textAlign: 'center',
            marginTop: '20px',
        },
        'charts-popover': {
            '& .dreamview-popover-inner': {
                padding: 0,
            },
        },
        'charts-container': {
            padding: theme.tokens.margin.speace2,
            height: '100%',
        },
        'fixed-left': {
            left: '0 !important',
        },
    }));

    return hoc();
}
