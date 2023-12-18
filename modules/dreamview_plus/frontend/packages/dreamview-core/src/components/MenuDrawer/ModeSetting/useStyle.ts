import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme, prop) => ({
        'mode-setting': {
            display: 'flex',
            flexDirection: 'column',
            height: '100%',
        },
        'mode-setting-container': {
            padding: theme.tokens.padding.speace3,
            height: prop?.height,
        },
        'modules-operation': {
            display: 'flex',
            justifyContent: 'space-between',
            marginBottom: theme.tokens.margin.speace2,
        },
        space20px: {
            marginBottom: '20px',
        },
        speace3: {
            marginBottom: theme.tokens.margin.speace3,
        },

        // Modules
        'modules-switch-container': {
            display: 'flex',
            flexWrap: 'wrap',
            justifyContent: 'space-between',
        },
        'modules-switch-item': {
            width: '47%',
            marginBottom: theme.tokens.margin.speace2,
            '& > label': {
                display: 'flex',
                alignItems: 'center',
            },
        },
        'modules-switch-text': {
            flex: 1,
            marginLeft: theme.tokens.margin.speace,
            fontSize: theme.tokens.font.size.regular,
            ...theme.util.textEllipsis,
            whiteSpace: 'nowrap',
        },

        resource: {
            marginBottom: '20px',
            '.dreamview-tabs-nav .dreamview-tabs-nav-list': {
                display: 'inline-flex',
                flex: 'none',
                background: theme.tokens.colors.background1,
            },
            '.dreamview-tabs-tab': {
                padding: '5px 16px',
                minWidth: '106px',
                justifyContent: 'center',
                margin: '0 !important',
            },
            '.dreamview-tabs-ink-bar': {
                display: 'none',
            },
            '.dreamview-tabs-tab.dreamview-tabs-tab-active .dreamview-tabs-tab-btn': {
                color: theme.tokens.colors.fontColor1,
            },
            '.dreamview-tabs-tab.dreamview-tabs-tab-active ': {
                backgroundColor: theme.tokens.colors.brand2,
                borderRadius: '6px',
            },
        },
    }));
    return hoc;
}

export function useCurrentResourceStyle() {
    const hoc = useMakeStyle((theme) => ({
        'current-resource-item': {
            height: '40px',
            fontSize: theme.tokens.font.size.regular,
            lineHeight: '40px',
            margin: '0 -14px',
            borderRadius: '8px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            cursor: 'pointer',
            backgroundColor: theme.tokens.colors.brand3,
            color: theme.tokens.colors.fontColor1,
            marginBottom: theme.tokens.margin.speace,
            '& .anticon': {
                marginRight: '14px',
                fontSize: theme.tokens.font.size.large,
            },
            '&:last-of-type': {
                marginBottom: '20px',
            },
        },
        name: {
            marginLeft: theme.tokens.margin.speace3,
            ...theme.util.textEllipsis,
        },
        empty: {
            textAlign: 'center',
            color: theme.tokens.colors.fontColor3,
            marginBottom: '20px',
            fontSize: theme.tokens.font.size.regular,
            img: {
                display: 'block',
                margin: '0 auto',
            },
        },
    }));

    return hoc();
}

export function useGuideContainerStyle() {
    const hoc = useMakeStyle((theme) => ({
        'guide-container': {
            margin: '-6px -16px 0',
            padding: '6px 16px 0',
        },
    }));

    return hoc();
}
