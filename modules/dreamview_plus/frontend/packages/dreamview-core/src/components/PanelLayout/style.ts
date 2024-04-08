import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'layout-root': {
            position: 'relative',
            height: '100%',
            marginLeft: '-1px',
        },
    }));
    return hoc();
}
