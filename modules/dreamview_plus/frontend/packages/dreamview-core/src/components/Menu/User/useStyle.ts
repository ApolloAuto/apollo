import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        title: {
            ...theme.tokens.typography.title,
        },
        login: {
            ...theme.tokens.typography.content,
            color: theme.tokens.colors.brand2,
            cursor: 'pointer',
            margin: `${theme.tokens.margin.speace} 0`,
        },
        userinfo: {
            display: 'flex',
            alignItems: 'center',
            marginBottom: '10px',
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
    return hoc();
}

export function useMenuItemStyle() {
    const hoc = useMakeStyle((theme) => ({
        menu: {
            padding: '12px 0',
            borderTop: '1px solid rgba(255,255,255,0.08)',
        },
        'menu-item': {
            ...theme.tokens.typography.content,
            lineHeight: '32px',
            color: theme.tokens.colors.fontColor2,
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
    return hoc();
}
