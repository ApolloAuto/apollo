import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'progress-container': {
        position: 'relative',
        width: '100%',
        padding: '4px 0',
        cursor: 'pointer',
        '& .progress-background': {
            backgroundColor: theme.components.bottomBar.progressBgColor,
        },
        '& .progress-inner': {
            height: '4px',
            ...theme.components.bottomBar.progressColorActiveColor,
        },
        '& .progress-pointer': {
            position: 'absolute',
            borderRadius: '12px',
            width: '12px',
            height: '12px',
            backgroundColor: theme.tokens.colors.brand3,
            border: '2px solid white',
            boxShadow: '0px 6px 6px 0px rgba(0,75,179,0.5)',
            top: '0px',
            '&::after': {
                content: '""',
                position: 'absolute',
                left: '-8px',
                top: '-8px',
                width: '24px',
                height: '24px',
                borderRadius: '50%',
                background: 'rgba(50,136,250,0.20)',
                transition: theme.tokens.transitions.easeInOut(),
            },
        },
    },
}));
