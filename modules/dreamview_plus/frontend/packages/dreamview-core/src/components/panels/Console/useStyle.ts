import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'panel-console-root': {
            padding: theme.tokens.padding.speace2,
            height: '100%',
            width: '100%',
        },
        'panel-console-inner': {
            minWidth: '244px',
        },
        'panel-console-monitor': {
            display: 'flex',
            marginBottom: '10px',
            '&:last-of-type': {
                marginBottom: '0',
            },
        },
        'panel-console-monitor-icon': {
            display: 'flex',
            marginTop: '2px',
            fontSize: theme.tokens.font.size.large,
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
            color: theme.tokens.colors.error,
        },
        info: {
            color: theme.tokens.colors.brand2,
        },
        warn: {
            color: theme.tokens.colors.warn,
        },
    }));
    return hoc();
}
