import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'source-name': {
            position: 'absolute',
            top: 18,
            left: 20,
            right: 20,
            ...theme.util.textEllipsis,
            whiteSpace: 'nowrap',
        },
    }));

    return hoc();
}
