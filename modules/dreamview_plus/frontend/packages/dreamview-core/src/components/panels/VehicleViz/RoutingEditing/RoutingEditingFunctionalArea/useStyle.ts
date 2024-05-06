import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle(areaProps: any) {
    const hoc = useMakeStyle((theme, props) => ({
        'routing-editing-function-area': {
            width: 32,
            position: 'absolute',
            top: 25,
            left: 24,
            color: '',
            ...theme.util.flex('column', 'space-around', 'center'),
        },
        'routing-editing-function-area__group': {
            background: theme.components.layerMenu.menuItemBg,
            borderRadius: 6,
            marginBottom: 10,
        },
        'routing-editing-functional__item': {
            width: 32,
            minWidth: 32,
            height: 32,
            minHeight: 32,
            background: theme.components.layerMenu.menuItemBg,
            borderRadius: 6,
            color: theme.components.layerMenu.menuItemColor,
            ...theme.util.flexCenterCenter,
            '&:hover': {
                cursor: 'pointer',
                background: theme.components.layerMenu.menuItemBg,
            },
        },
        'hover-color-change': {
            '&:hover': {
                color: theme.components.layerMenu.menuItemHoverColor,
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

        'routing-editing-functional__item--disable': {
            cursor: 'not-allowed',
            '&:hover': {
                cursor: 'not-allowed',
            },
        },
    }));
    return hoc(areaProps);
}
