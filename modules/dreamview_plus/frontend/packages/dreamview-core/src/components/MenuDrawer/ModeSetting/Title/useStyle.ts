import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'mode-setting-title': {
            ...theme.tokens.typography.title,
            paddingLeft: '10px',
            position: 'relative',
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
        },
        'mode-setting-title-expend': {
            marginBottom: 0,
        },
        'mode-setting-collapse': {
            border: 'none',
            backgroundColor: 'transparent',
            '&.dreamview-collapse > .dreamview-collapse-item:last-child > .dreamview-collapse-header': {
                padding: 0,
                position: 'relative',
                color: theme.tokens.colors.fontColor2,
                marginBottom: theme.tokens.margin.speace2,
                border: 'none',
            },
            '& .rc-collapse-item': {
                border: 'none',
            },
            '& .dreamview-collapse-content': {
                backgroundColor: 'transparent',
                padding: 0,
            },
            '& .dreamview-collapse-content > .dreamview-collapse-content-box': {
                margin: 0,
                paddingBlock: '0 !important',
                padding: 0,
            },
            '& .dreamview-collapse-expand-icon': {
                position: 'absolute',
                right: 0,
                top: 0,
                bottom: 0,
                margin: 'auto 0',
                paddingRight: '0px !important',
                ...theme.util.flexCenterCenter,
                '& svg': {
                    transition: theme.tokens.transitions.easeInOut(),
                    fontSize: '16px',
                },
            },
        },
    }));
    return hoc();
}
