import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'moniter-item-container': {
        backgroundColor: '#282D38',
        color: theme.tokens.colors.fontColor5,
        borderRadius: '6px',
    },
    'moniter-item-title': {
        ...theme.tokens.typography.sideText,
        lineHeight: '32px',
        textAlign: 'center',
        background: '#343C4D',
        borderRadius: '6px 6px 0px 0px',
    },
    'moniter-item-content': {
        ...theme.tokens.typography.sideText,
        padding: `12px ${theme.tokens.padding.speace2}`,
        color: theme.tokens.colors.fontColor5,
        display: 'flex',
        justifyContent: 'space-around',
    },
    'moniter-item-left': {
        minWidth: '25%',
        maxWidth: '120px',
        whiteSpace: 'nowrap',
    },
    'moniter-item-mid': {
        minWidth: '50%',
        padding: '0 24px',
        '& > div': {
            ...theme.util.textEllipsis,
        },
    },
    'moniter-item-right': {
        minWidth: '25%',
        '& > div': {
            ...theme.util.textEllipsis,
        },
    },
    time: {
        width: '20%',
    },
    scenarioPluginType: {
        width: '40%',
    },
    stagePluginType: {
        width: '40%',
    },
}));
