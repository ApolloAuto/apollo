import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles(() => ({
    'functional-loop-con': {
        width: '257px',
        padding: '16px 0px 21px 0px',
        color: '#A6B5CC',
        fontSize: '14px',
        fontWeight: '400',
        fontFamily: 'PingFangSC-Regular',
    },
    'functional-loop-switch': {
        width: '160px',
        height: '20px',
        margin: '0px 0px 0px 32px',
        display: 'flex',
        justifyContent: 'space-around',
        alignItems: 'center',
    },
    'functional-loop-switch-help': {
        '& .dreamview-popover-arrow::before': {
            background: 'rgba(40, 43, 54) !important',
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(40, 43, 54) !important',
        },
        '& .dreamview-popover-inner-content': {
            width: '302px',
            color: '#FFFFFF',
            fontSize: '14px',
            fontWeight: '400',
            padding: '5px 10px',
            borderRadius: '6px',
            fontFamily: 'PingFangSC-Regular',
            background: 'rgba(80, 88, 102, 0.8)',
        },
    },
    'functional-loop-switch-remind': {
        position: 'relative',
        top: '2px',
    },
    'functional-loop-input': {
        height: '32px',
        margin: '16px 0px 0px 20px',
        display: 'flex',
        alignItems: 'center',
    },
    'functional-loop-input-text': {
        marginRight: '6px',
    },
}));