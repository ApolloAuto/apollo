import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'mode-setting-title': {
            ...theme.tokens.typography.title,
            paddingLeft: '10px',
            position: 'relative',
            cursor: 'pointer',
            marginBottom: theme.tokens.margin.speace,
            '&::after': {
                position: 'absolute',
                width: '2px',
                height: '12px',
                background: theme.tokens.colors.brand2,
                content: '""',
                left: 0,
                top: 0,
                bottom: 0,
                margin: 'auto 0',
            },
            '& .anticon': {
                position: 'absolute',
                right: 0,
                top: '4px',
                transform: 'rotate(0)',
                transition: theme.tokens.transitions.easeInOut(),
            },
        },
        'mode-setting-icon-active': {
            '& .anticon': {
                transform: 'rotate(180deg)',
            },
        },
        'mode-setting-expend': {
            transition: theme.tokens.transitions.easeInOut(),
            height: 'auto',
            overflow: 'hidden',
            padding: '0 14px',
            margin: '0 -14px',
        },
    }));
    return hoc();
}
