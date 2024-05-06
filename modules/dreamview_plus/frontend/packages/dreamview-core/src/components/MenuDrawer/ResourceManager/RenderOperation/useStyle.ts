import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'source-operate': {},
        'source-operate-icon': {
            fontSize: theme.tokens.font.size.large,
            cursor: 'pointer',
            marginRight: '32px',
        },
    }));

    return hoc();
}
