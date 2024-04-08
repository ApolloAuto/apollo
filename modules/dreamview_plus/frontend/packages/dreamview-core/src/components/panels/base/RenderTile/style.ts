import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'panel-item-container': {
            height: '100%',
            display: 'flex',
            flexDirection: 'column',
            position: 'relative',
        },
        'panel-item-inner': {
            flex: 1,
            overflow: 'hidden',
        },
        dragging: {
            border: '1px solid rgb(41,125,236)',
        },
        'panel-error': {
            display: 'flex',
            height: '100%',
            alignItems: 'center',
            textAlign: 'center',
            color: theme.tokens.colors.fontColor1,
            '& > div': {
                flex: 1,
            },
            span: {
                color: theme.tokens.colors.brand3,
                cursor: 'pointer',
            },
            img: {
                width: '80px',
                height: '80px',
            },
        },
    }));
    return hoc();
}
