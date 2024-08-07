import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dv-root': {
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        display: 'flex',
    },
    'dv-layout-menu': {
        width: 64,
        height: '100%',
        background: theme.tokens.backgroundColor.main,
        borderRight: `1px solid ${theme.tokens.colors.divider2}`,
        position: 'relative',
        zIndex: 2,
    },
    'dv-layout-content': {
        flex: 1,
        display: 'flex',
        flexDirection: 'column',
        position: 'relative',
        height: '100vh',
    },
    'dv-layout-window': {
        flex: 1,
        display: 'flex',
        position: 'relative',
        background: theme.tokens.colors.background3,
        overflow: 'hidden',
    },
    'dv-layout-panellayout': {
        flex: 1,
        position: 'relative',
        background: theme.tokens.backgroundColor.transparent,
    },
    'dv-layout-playercontrol': {
        height: 0,
        transition: theme.tokens.transitions.easeIn('height'),
        background: theme.components.bottomBar.bgColor,
        position: 'relative',
        zIndex: 1,
        overflow: 'hidden',
        boxShadow: 'none',
        border: 'none',
    },
    'dv-layout-playercontrol-active': {
        height: 64,
        boxShadow: theme.components.bottomBar.boxShadow,
        borderTop: theme.components.bottomBar.border,
    },
}));
