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
            '& .dreamview-tabs-tab': {
                padding: '17px 4px',
            },
            '&.dreamview-tabs .dreamview-tabs-tab': {
                fontSize: theme.tokens.font.size.large,
                padding: '0 4px',
                height: '56px',
                lineHeight: '56px',
                width: '80px',
                justifyContent: 'center',
            },
            '&.dreamview-tabs .dreamview-tabs-nav .dreamview-tabs-nav-list': {
                background: '#242933',
                borderRadius: '10px',
            },
            '& .dreamview-tabs-ink-bar': {
                height: '1px !important',
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
            [`&  tr:nth-of-type(${prop.activeIndex}):hover td`]: {
                background: theme.tokens.colors.brand3,
            },
            [`&  tr:nth-of-type(${prop.activeIndex}) td`]: {
                background: theme.tokens.colors.brand3,
                color: theme.tokens.colors.fontColor1,
                '& .download-active': {
                    color: theme.tokens.colors.fontColor1,
                },
            },
        },
    }));
}
