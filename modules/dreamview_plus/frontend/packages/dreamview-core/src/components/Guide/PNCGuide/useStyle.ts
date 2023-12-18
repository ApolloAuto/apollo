import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme, props) => ({
        'dv-pnc-guide-skip': {
            ...theme.tokens.typography.content,
            color: theme.tokens.font.color.main,
        },
        'dv-pnc-guide-current-value': {
            ...theme.tokens.typography.content,
            fontSize: '18px',
            color: '#FFFFFF',
        },
        'dv-pnc-guide-all-value': {
            ...theme.tokens.typography.content,
            color: '#D3DCEB',
        },

        'dv-pnc-guide-back-node': {
            width: '72px',
            height: '40px',
            ...theme.tokens.typography.content,
            color: '#FFFFFF',
            backgroundColor: theme.tokens.backgroundColor.mainLight,
            border: '1px solid rgba(124,136,153,1)',
            borderRadius: '8px',
            '&:hover': {
                color: '#3288FA',
                border: '1px solid #3288FA',
                backgroundColor: 'transparent',
            },
        },

        'dv-pnc-guide-next-node': {
            width: '72px',
            height: '40px',
            ...theme.tokens.typography.content,
            color: '#FFFFFF',
            backgroundColor: theme.colors.blue6,
            borderRadius: '8px',
        },
        'dv-pnc-guide-close-node': {
            width: '72px',
            height: '40px',
            ...theme.tokens.typography.content,
            color: '#FFFFFF',
            backgroundColor: theme.colors.blue6,
            borderRadius: '8px',
        },
    }));
    return hoc();
}
