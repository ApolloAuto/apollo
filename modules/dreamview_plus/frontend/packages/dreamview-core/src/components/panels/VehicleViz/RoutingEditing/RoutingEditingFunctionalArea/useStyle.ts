import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'routing-editing-function-area': {
            width: 32,
            position: 'absolute',
            top: 25,
            left: 24,
            ...theme.util.flex('column', 'space-around', 'center'),
        },
        'routing-editing-function-area__group': {
            background: '#343C4D',
            borderRadius: 6,
            marginBottom: 10,
        },
        'routing-editing-functional__item': {
            width: 32,
            minWidth: 32,
            height: 32,
            minHeight: 32,
            background: '#343C4D',
            borderRadius: 6,
            ...theme.util.flexCenterCenter,
            '&:hover': {
                cursor: 'pointer',
                background: '#38465A',
            },
        },
        'hover-color-change': {
            '&:hover': {
                color: '#fff',
            },
        },
        'routing-editing-functional__item--active': {
            color: '#3388FA',
        },
        'routing-editing-functional__icon': {},

        'custom-popover-ordinary': {
            '& .dreamview-popover-inner, & .dreamview-popover-arrow::after': {
                background: 'rgba(80, 88, 102, 0.8) !important',
            },
            '& .dreamview-popover-arrow::after': {
                background: 'rgba(80, 88, 102, 0.8) !important',
            },
        },
        'custom-popover-functinal': {
            '& .dreamview-popover-inner,& .dreamview-popover-arrow::before, & .dreamview-popover-arrow::after': {
                color: '#A6B5CC !important',
                background: 'rgba(40, 43, 54) !important',
            },
            '& .dreamview-popover-arrow::before': {
                background: 'rgba(40, 43, 54) !important',
            },
            '& .dreamview-popover-arrow::after': {
                background: 'rgba(40, 43, 54) !important',
            },
        },
    }));
    return hoc();
}
