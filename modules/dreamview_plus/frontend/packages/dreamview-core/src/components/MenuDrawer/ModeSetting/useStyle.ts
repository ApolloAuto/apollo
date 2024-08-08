import { makeStylesWithProps, makeStyles } from '@dreamview/dreamview-theme';

export default makeStylesWithProps<{
    height?: string;
}>()((theme, prop) => ({
    'mode-setting': {
        display: 'flex',
        flexDirection: 'column',
        height: '100%',
        '& .dreamview-select': {
            width: '100% !important',
        },
    },
    'mode-setting-container': {
        padding: `12px ${theme.tokens.padding.speace3}`,
        height: prop?.height,
    },
    'modules-operation': {
        display: 'flex',
        justifyContent: 'space-between',
        marginBottom: theme.tokens.margin.speace2,
    },
    space12px: {
        marginBottom: '12px',
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
    },
}));

export const useCurrentResourceStyle = makeStyles((theme) => ({
    'current-resource-item': {
        height: '32px',
        fontSize: theme.tokens.font.size.regular,
        lineHeight: '32px',
        borderRadius: '8px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        cursor: 'pointer',
        backgroundColor: theme.components.sourceItem.activeBgColor,
        color: theme.components.sourceItem.activeColor,
        marginBottom: theme.tokens.margin.speace,
        padding: '12px',
        '& .anticon': {
            color: theme.components.sourceItem.activeIconColor,
            fontSize: theme.tokens.font.size.large,
        },
        '&:last-of-type': {
            marginBottom: '20px',
        },
    },
    name: {
        ...theme.util.textEllipsis,
    },
    empty: {
        textAlign: 'center',
        color: theme.tokens.colors.fontColor4,
        marginBottom: '20px',
        fontSize: theme.tokens.font.size.regular,
        img: {
            display: 'block',
            margin: '0 auto',
        },
    },
}));

export const useGuideContainerStyle = makeStyles(() => ({
    'guide-container': {
        margin: '-6px -16px 0',
        padding: '6px 16px 0',
    },
}));
