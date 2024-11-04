import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'functional-initial-con': {
        display: 'flex',
    },
    'functional-initial-every-icon-con': {
        cursor: 'not-allowed',
    },
    'functional-initial-disable': {
        '& .functional-initial-every-icon-disable': {
            color: '#40454D',
            pointerEvents: 'none',
        },
    },
    'functional-initial-popover': {
        '& .dreamview-popover-inner-content': {
            padding: '5px 10px',
            color: '#FFFFFF',
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            fontWeight: '400',
        },
    },
    'functional-initial-every-icon': {
        width: '32px',
        height: '32px',
        color: '#A6B5CC',
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        '&:hover': {
            color: '#D8D8D8',
            background: 'rgba(115,193,250,0.08)',
        },
    },
}));
