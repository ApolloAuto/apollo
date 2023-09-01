import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme, props) => ({
        'dv-layout-menudrawer-item': {
            position: 'relative',
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

    return hoc;
}
