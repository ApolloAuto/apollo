import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'mode-setting-divider': {
            borderBottom: `1.5px solid ${theme.tokens.divider.color.strong}`,
            marginBottom: theme.tokens.margin.speace2,
        },
    }));
    return hoc();
}
