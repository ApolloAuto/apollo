import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'moniter-item-container': {
            backgroundColor: '#282D38',
            color: theme.tokens.colors.fontColor5,
            borderRadius: '6px',
        },
        'moniter-item-title': {
            ...theme.tokens.typography.sideText,
            lineHeight: '32px',
            textAlign: 'center',
            background: '#343C4D',
            borderRadius: '6px 6px 0px 0px',
        },
        'moniter-item-chart': {
            width: '100%',
            height: '300px',
        },
    }))();

    return hoc;
}
