import { makeStyles, makeStylesWithProps } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
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

export const useTableHover = makeStylesWithProps<number>()((theme, activeIndex) => ({
    'table-active': {
        [`&  tr:nth-of-type(${activeIndex + 1}):hover td`]: {
            background: theme.components.table.activeBgColor,
        },
        [`&  tr:nth-of-type(${activeIndex + 1}) td`]: {
            background: theme.components.table.activeBgColor,
            color: theme.tokens.colors.fontColor5,
            '& .download-active': {
                color: theme.tokens.colors.fontColor5,
            },
        },
    },
}));

export const useTitleTrangle = makeStyles((theme) => ({
    'title-operate': {
        display: 'inline-flex',
        alignItems: 'center',
    },
    'title-triangle': {
        marginLeft: '8px',
        // position: 'absolute',
        width: 0,
        height: 0,
        // top: '6px',
        // right: '-16px',
        borderLeft: '5px solid transparent',
        borderRight: '5px solid transparent',
        borderBottom: '6px solid #D8D8D8',
    },
    'operate-container': {
        margin: '-12px 0',
    },
    'operate-item': {
        cursor: 'pointer',
        textAlign: 'left',
        height: '32px',
        lineHeight: '32px',
        whiteSpace: 'nowrap',
        padding: '0 12px',
        color: theme.tokens.colors.fontColor6,
        '&:hover': {
            color: theme.tokens.colors.brand3,
        },
    },
    'operate-item-active': {
        color: theme.tokens.colors.brand3,
    },
}));
