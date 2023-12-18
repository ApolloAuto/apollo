import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function usePageLayoutStyles() {
    const hoc = useMakeStyle((theme) => ({
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
            background: '#383C4D',
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
            background: theme.tokens.colors.background2,
            borderTop: '1px solid #252833',
            marginTop: '-1px',
            position: 'relative',
            overflow: 'hidden',
        },
        'dv-layout-playercontrol-active': {
            height: 64,
        },
    }));
    return hoc();
}
