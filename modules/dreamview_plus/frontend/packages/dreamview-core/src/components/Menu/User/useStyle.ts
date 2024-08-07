import { makeStyles, makeStylesWithProps } from '@dreamview/dreamview-theme';

const useHoc = makeStylesWithProps<{ themeText: string }>()((theme, prop) => ({
    title: {
        ...theme.tokens.typography.title,
        color: prop.themeText === 'drak' ? 'white' : '#232A33',
    },
    login: {
        ...theme.tokens.typography.content,
        color: theme.tokens.colors.brand3,
        cursor: 'pointer',
        margin: `${theme.tokens.margin.speace} 0 ${theme.tokens.margin.speace2}`,
    },
    userinfo: {
        display: 'flex',
        alignItems: 'center',
        marginBottom: '10px',
        color: prop.themeText === 'drak' ? 'white' : '#232A33',
    },
    avatar: {
        marginRight: theme.tokens.margin.speace,
        '& img': {
            display: 'block',
            width: '32px',
            height: '32px',
            borderRadius: '50%',
        },
    },
    displayname: {
        ...theme.tokens.typography.title1,
        fontSize: theme.tokens.font.size.huge,
    },
    'avatar-default': {
        width: '32px',
        height: '32px',
        backgroundSize: 'cover',
        backgroundRepeat: 'no-repeat',
    },
}));

export default function useStyle(themeText: any) {
    return useHoc({ themeText });
}

export const useMenuItemStyle = makeStyles((theme) => ({
    menu: {
        padding: `${theme.tokens.padding.speace} 0`,
        borderTop: '1px solid rgba(255,255,255,0.08)',
    },
    'menu-item': {
        ...theme.tokens.typography.content,
        lineHeight: '32px',
        color: theme.tokens.colors.fontColor6,
        cursor: 'pointer',
        whiteSpace: 'nowrap',
        '&:hover': {
            color: theme.tokens.font.reactive.mainHover,
        },
        '&:active': {
            color: theme.tokens.font.reactive.mainActive,
        },
        '& .anticon': {
            fontSize: theme.tokens.font.size.large,
            marginRight: theme.tokens.margin.speace,
        },
    },
}));
