import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'routing-editing-op-area': {
        width: 192,
        height: 32,
        color: '#A6B5CC',
        borderRadius: 6,
        position: 'absolute',
        top: 24,
        left: 24,
        ...theme.util.flex('row', 'space-between', 'center'),
    },
    'routing-editing-btn': {
        width: 144,
        height: 32,
        borderRadius: 6,
        background: '#343C4D',
        ...theme.util.flexCenterCenter,
        '&:hover': {
            cursor: 'pointer',
            background: '#38465A',
            color: '#fff',
        },
    },
    'routing-editing-btn__icon': {
        marginRight: 6,
    },
    'routing-editing-btn__text': {
        fontFamily: 'PingFangSC-Regular',
        fontSize: 14,
    },
    'common-routing-editing-btn': {
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
            color: '#fff',
        },
    },
    'common-routing-editing-btn__icon': {},
    'routing-editing-disable': {
        cursor: 'not-allowed',
        '& .routing-editing-btn-disable': {
            color: '#515761',
            pointerEvents: 'none',
            background: '#383D47',
        },
        '& .common-routing-editing-btn-disbale': {
            color: '#515761',
            pointerEvents: 'none',
            background: '#383D47',
        },
    },

    'common-routing-popover-ordinary': {
        '& .dreamview-popover-inner, & .dreamview-popover-arrow::after': {
            background: 'rgba(80, 88, 102, 0.8) !important',
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(80, 88, 102, 0.8) !important',
        },
    },

    'common-routing-popover-functional': {
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
