import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'menu-drawer-title': {
            padding: '0 24px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            height: '78px',
            ...theme.tokens.typography.title1,
        },
        'menu-drawer-title-icclose': {
            fontSize: theme.tokens.font.size.large,
            cursor: 'pointer',
            padding: '10px',
            marginRight: '-10px',
            ...theme.util.func.textReactive(
                theme.tokens.font.reactive.mainHover,
                theme.tokens.font.reactive.mainActive,
            ),
            '& .anticon': {
                display: 'block',
            },
        },
        'menu-drawer-title-border': {
            borderBottom: `1px solid ${theme.tokens.colors.divider3}`,
        },
    }));
    return hoc();
}
