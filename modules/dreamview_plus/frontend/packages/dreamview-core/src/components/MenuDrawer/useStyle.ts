import { makeStylesWithProps } from '@dreamview/dreamview-theme';

export default makeStylesWithProps<{ width: string; backgroundColor: string }>()((theme, props) => ({
    'dv-layout-menudrawer-item': {
        position: 'relative',
        height: '100%',
    },
    hidden: {
        position: 'absolute',
        zIndex: '-1',
    },
    'dv-layout-menudrawer': {
        width: props.width,
        background: props.backgroundColor || theme.tokens.backgroundColor.main,
        // transition: theme.tokens.transitions.easeIn(),
        color: theme.tokens.font.color.mainLight,
    },
    'dv-layout-menudrawer-border': {
        borderRight: `1px solid ${theme.tokens.colors.divider3}`,
    },
}));
