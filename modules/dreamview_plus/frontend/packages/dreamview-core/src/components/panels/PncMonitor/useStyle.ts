import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'pnc-container': {
            padding: theme.tokens.padding.speace2,
            position: 'relative',
            height: '100%',
        },
        'pnc-monitor-filter-popover': {
            '& .dreamview-popover-title': {
                padding: `${theme.tokens.padding.speace} ${theme.tokens.padding.speace3}`,
                ...theme.tokens.typography.title,
                borderBottom: '1px solid #484C59',
                marginBottom: 0,
            },
            '&.dreamview-popover .dreamview-popover-inner': {
                backgroundColor: '#353947',
                padding: 0,
            },
            '& .dreamview-popover-inner-content': {
                padding: `${theme.tokens.padding.speace2} ${theme.tokens.padding.speace3}`,
            },
        },
        'pnc-monitor-filter': {
            color: theme.tokens.colors.fontColor2,
            position: 'absolute',
            right: theme.tokens.margin.speace2,
            top: theme.tokens.margin.speace3,
            '&:hover': {
                color: theme.tokens.font.reactive.mainHover,
            },
            '&:active': {
                color: theme.tokens.font.reactive.mainActive,
            },
        },
        'pnc-monitor-filter-content': {
            display: 'flex',
            flexWrap: 'wrap',
            maxWidth: '430px',
        },
        'pnc-monitor-filter-item': {
            width: '50%',
            ...theme.util.textEllipsis,
            ...theme.tokens.typography.content,
            color: theme.tokens.colors.fontColor2,
            marginBottom: '12px',
            '& .dreamview-check-box-wrapper': {
                marginRight: theme.tokens.margin.speace,
            },
        },
        tabs: {
            marginBottom: '20px',
            '.dreamview-tabs-nav .dreamview-tabs-nav-list': {
                display: 'inline-flex',
                flex: 'none',
                background: theme.tokens.colors.background1,
            },
            '.dreamview-tabs-tab': {
                padding: '5px 0',
                width: '100px',
                justifyContent: 'center',
                margin: '0 !important',
            },
            '.dreamview-tabs-ink-bar': {
                display: 'none',
            },
            '.dreamview-tabs-tab.dreamview-tabs-tab-active .dreamview-tabs-tab-btn': {
                color: theme.tokens.colors.fontColor1,
                ...theme.tokens.typography.content,
            },
            '.dreamview-tabs-tab.dreamview-tabs-tab-active ': {
                backgroundColor: theme.tokens.colors.brand2,
                borderRadius: '6px',
            },
        },
    }));

    return hoc();
}
