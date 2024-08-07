import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
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
        color: theme.tokens.colors.fontColor5,
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
        color: theme.tokens.colors.fontColor5,
        marginBottom: '12px',
        '& .dreamview-check-box-wrapper': {
            marginRight: theme.tokens.margin.speace,
        },
    },
    tabs: {
        marginBottom: '20px',
        '.dreamview-tabs-tab.dreamview-tabs-tab-active .dreamview-tabs-tab-btn': {
            ...theme.tokens.typography.content,
        },
    },
}));
