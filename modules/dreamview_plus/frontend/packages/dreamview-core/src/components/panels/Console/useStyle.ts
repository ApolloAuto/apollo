import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'panel-console-root': {
        padding: `4px ${theme.tokens.padding.speace2}`,
        height: '100%',
        width: '100%',
    },
    'panel-console-inner': {
        minWidth: '244px',
    },
    'panel-console-monitor': {
        display: 'flex',
        marginBottom: theme.tokens.margin.speace,
        '&:last-of-type': {
            marginBottom: '0',
        },
    },
    'panel-console-monitor-icon': {
        display: 'flex',
        marginTop: '-1px',
        fontSize: theme.components.panelConsole.iconFontSize,
    },
    'panel-console-monitor-text': {
        marginLeft: theme.tokens.margin.speace,
        marginRight: theme.tokens.margin.speace2,
        ...theme.tokens.typography.content,
        flex: 1,
    },
    'panel-console-monitor-time': {
        whiteSpace: 'nowrap',
    },
    'panel-console-monitor-item': {},
    error: {
        color: theme.tokens.colors.error2,
    },
    info: {
        color: theme.tokens.colors.brand3,
    },
    warn: {
        color: theme.tokens.colors.warn2,
    },
}));
