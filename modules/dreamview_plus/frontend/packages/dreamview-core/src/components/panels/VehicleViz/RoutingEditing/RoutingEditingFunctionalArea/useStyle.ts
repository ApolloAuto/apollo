import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'routing-editing-function-area': {
        width: 40,
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
        width: 40,
        minWidth: 40,
        height: 40,
        minHeight: 40,
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
    'routing-editing-functional__icon': {
        fontSize: 20,
    },

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
