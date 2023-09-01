import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'table-background': {
            overflow: 'hidden',
            borderRadius: '10px',
            background: '#282B36',
        },
    }));

    return hoc();
}
