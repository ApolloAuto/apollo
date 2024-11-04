import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'panel-module-delay-root': {
        padding: `4px ${theme.tokens.padding.speace2}`,
        height: '100%',
        width: '100%',
    },
    'panel-module-delay-scroll': {
        minWidth: '244px',
    },
    'panel-module-delay-item': {
        display: 'flex',
        ...theme.tokens.typography.content,
        color: theme.tokens.font.color.main,
        marginBottom: theme.tokens.margin.speace,
        '&:last-of-type': {
            marginBottom: 0,
        },
    },
    name: {
        whiteSpace: 'nowrap',
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        flex: 1,
        marginRight: theme.tokens.margin.speace,
    },
    time: {
        whiteSpace: 'nowrap',
    },
    error: {
        color: theme.tokens.font.color.error,
    },
}));
