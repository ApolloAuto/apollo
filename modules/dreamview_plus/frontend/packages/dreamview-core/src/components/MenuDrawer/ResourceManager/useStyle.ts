import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'profile-manager-container': {
            padding: `0 ${theme.tokens.margin.speace3} ${theme.tokens.margin.speace3} ${theme.tokens.margin.speace3}`,
        },
        'profile-manager-tab-container': {
            position: 'relative',
        },
        'profile-manager-tab': {
            '& .dreamview-tabs-nav-wrap .dreamview-tabs-nav-list .dreamview-tabs-tab': {
                margin: '0 20px !important',
                background: theme.components.meneDrawer.tabBackgroundColor,
                '& .dreamview-tabs-tab-btn': {
                    color: theme.components.meneDrawer.tabColor,
                },
            },
            '&.dreamview-tabs .dreamview-tabs-tab': {
                fontSize: theme.tokens.font.size.large,
                padding: '0 4px',
                height: '56px',
                lineHeight: '56px',
                minWidth: '80px',
                justifyContent: 'center',
            },
            '& .dreamview-tabs-tab.dreamview-tabs-tab-active': {
                background: theme.components.meneDrawer.tabActiveBackgroundColor,
                '& .dreamview-tabs-tab-btn': {
                    color: theme.components.meneDrawer.tabActiveColor,
                },
            },
            '&.dreamview-tabs .dreamview-tabs-nav .dreamview-tabs-nav-list': {
                background: theme.components.meneDrawer.tabBackgroundColor,
                borderRadius: '10px',
            },
            '& .dreamview-tabs-ink-bar': {
                height: '1px !important',
                display: 'block !important',
            },
        },

        'profile-manager-tab-select': {
            display: 'flex',
            alignItems: 'center',
            position: 'absolute',
            zIndex: 1,
            right: 0,
            top: '8px',
            ...theme.tokens.typography.content,
            '.dreamview-select': {
                width: '200px !important',
            },
        },
    }));

    return hoc();
}

export function useTableHover() {
    return useMakeStyle((theme, prop) => ({
        'table-active': {
            [`&  tr:nth-of-type(${prop.activeIndex + 1}):hover td`]: {
                background: theme.components.table.activeBgColor,
            },
            [`&  tr:nth-of-type(${prop.activeIndex + 1}) td`]: {
                background: theme.components.table.activeBgColor,
                color: theme.tokens.colors.fontColor5,
                '& .download-active': {
                    color: theme.tokens.colors.fontColor5,
                },
            },
        },
    }));
}
