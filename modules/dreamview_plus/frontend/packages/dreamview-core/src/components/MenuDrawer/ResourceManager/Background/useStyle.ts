import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'table-background': {
            overflow: 'hidden',
            borderRadius: '10px',
            background: theme.components.table.bodyBgColor,
        },
    }));

    return hoc();
}
