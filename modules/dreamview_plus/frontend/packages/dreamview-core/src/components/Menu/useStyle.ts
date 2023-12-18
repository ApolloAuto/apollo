import { useMakeStyle } from '@dreamview/dreamview-theme';
import tinycolor from 'tinycolor2';
import icAddDesktopShortcut from '@dreamview/dreamview-core/src/assets/ic_add_desktop_shortcut.png';
import icAddDesktopShortcutHover from '@dreamview/dreamview-core/src/assets/ic_add_desktop_shortcut_hover.png';
import logo from '@dreamview/dreamview-core/src/assets/dreamview.png';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'menu-container': {
            height: '100%',
            display: 'flex',
            flexDirection: 'column',
            justifyContent: 'space-between',
            color: theme.tokens.font.color.mainLight,
            borderRight: `1px solid ${theme.tokens.colors.divider3}`,
        },
        'menu-item': {
            height: 50,
            margin: '14px 0',
            cursor: 'pointer',
            ...theme.util.flexCenterCenter,
            borderLeft: `${theme.tokens.divider.width.large}px solid ${theme.tokens.colors.transparent}`,
            '&:hover': {
                color: theme.tokens.colors.brand2,
            },
            '&:hover .add-desktop-hover': {
                backgroundColor: tinycolor(theme.tokens.colors.brand2).setAlpha(0.2).toRgbString(),
                borderColor: '#559CFA',
            },
        },
        active: {
            color: theme.tokens.colors.brand2,
            backgroundColor: tinycolor(theme.tokens.colors.brand2).setAlpha(0.15).toRgbString(),
            borderLeft: `${theme.tokens.divider.width.large}px solid ${theme.tokens.colors.brand2}`,
            '&:hover': {
                color: theme.tokens.colors.brand2,
            },
        },
        logo: {
            height: 56,
            background: `url(${logo})`,
            backgroundSize: '90%',
            backgroundRepeat: 'no-repeat',
            backgroundPosition: 'center',
        },
        'add-desktop': {
            height: '34px',
            width: '34px',
            backgroundImage: `url(${icAddDesktopShortcut})`,
            backgroundSize: '34px',
            backgroundRepeat: 'no-repeat',
            border: '1px solid transparent',
            borderRadius: '50%',
        },
        'menu-item-popover': {
            ...theme.tokens.typography.content,
            color: theme.tokens.font.color.white,
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
            '& .dreamview-popover-inner': {
                padding: `${theme.tokens.padding.speace2} ${theme.tokens.padding.speace3} 4px !important`,
            },
            '& .dreamview-popover-content': {
                width: '268px',
            },
        },
        popoverBg: {
            '& .dreamview-popover-content .dreamview-popover-inner': {
                padding: '5px 10px',
                background: 'rgba(61,67,78,0.80)',
            },
            '& .dreamview-popover-arrow::before': {
                background: 'rgba(61,67,78,0.80)',
            },
            '& .dreamview-popover-arrow::after': {
                display: 'none',
            },
        },
    }));
    return hoc();
}
