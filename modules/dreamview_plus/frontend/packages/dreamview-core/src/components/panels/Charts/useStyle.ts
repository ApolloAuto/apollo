import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'charts-operation': {
        textAlign: 'center',
        marginTop: '20px',
    },
    'charts-popover': {
        '& .dreamview-popover-inner': {
            padding: 0,
        },
    },
    'charts-container': {
        padding: theme.tokens.padding.speace2,
        height: '100%',
    },
    'fixed-left': {
        left: '0 !important',
    },
}));
