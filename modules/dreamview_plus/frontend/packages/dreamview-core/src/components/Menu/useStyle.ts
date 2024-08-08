import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import tinycolor from 'tinycolor2';
import { useImagePrak } from '@dreamview/dreamview-ui';

const useHoc = makeStylesWithProps<{
    logo: string;
    icAddDesktopShortcut: string;
}>()((theme, props) => ({
    'menu-container': {
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        justifyContent: 'space-between',
        color: theme.tokens.font.color.mainLight,
    },
    'menu-item': {
        height: 50,
        margin: '14px 0',
        cursor: 'pointer',
        ...theme.util.flexCenterCenter,
        borderLeft: `${theme.tokens.divider.width.large}px solid ${theme.tokens.colors.transparent}`,
        '&:hover': {
            color: theme.tokens.colors.brand3,
        },
        '&:hover .add-desktop-hover': {
            backgroundColor: tinycolor(theme.tokens.colors.brand3).setAlpha(0.2).toRgbString(),
            borderColor: '#559CFA',
        },
    },
    active: {
        color: theme.tokens.colors.brand3,
        backgroundColor: tinycolor(theme.tokens.colors.brand3).setAlpha(0.15).toRgbString(),
        borderLeft: `${theme.tokens.divider.width.large}px solid ${theme.tokens.colors.brand3}`,
        '&:hover': {
            color: theme.tokens.colors.brand3,
        },
    },
    logo: {
        height: 56,
        background: `url(${props.logo})`,
        backgroundSize: '34px',
        backgroundRepeat: 'no-repeat',
        backgroundPosition: 'center',
    },
    'add-desktop': {
        height: '34px',
        width: '34px',
        backgroundImage: `url(${props.icAddDesktopShortcut})`,
        backgroundSize: '34px',
        backgroundRepeat: 'no-repeat',
        border: '1px solid transparent',
        borderRadius: '50%',
    },
    'menu-item-popover': {
        ...theme.tokens.typography.content,
        color: theme.tokens.font.color.white,
    },
    'theme-btn': {
        width: '20px',
        height: '20px',
        borderRadius: '20px',
        fontSize: '20px',
    },
    'change-theme': {
        color: theme.components.menu.themeBtnColor,
        '&:hover': {
            color: theme.components.menu.themeHoverColor,
        },
        '& svg': {
            display: 'block',
        },
    },
    'menu-item-image': {
        svg: {
            width: '24px',
            height: '24px',
        },
    },
    'menu-item-avatar-image': {
        width: '24px',
        height: '24px',
        borderRadius: '50%',
        backgroundSize: 'cover',
        backgroundRepeat: 'no-repeat',
    },
    popover: {
        '& .dreamview-operate-popover-inner-content': {
            padding: '0 12px',
            marginBottom: '-20px',
        },
        '& .dreamview-operate-popover-content': {
            width: '268px',
        },
    },
}));

export default function useStyle() {
    const [icAddDesktopShortcut, logo] = useImagePrak(['ic_add_desktop_shortcut', 'dreamview']);
    return useHoc({ icAddDesktopShortcut, logo });
}
