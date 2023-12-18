import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'profile-table': {
            '& table': {
                width: '100%',
                borderCollapse: 'separate',
                borderSpacing: 0,
            },
            '& thead': {
                borderRadius: '10px 10px 0 0',
                textAlign: 'left',
            },
            '& thead th': {
                padding: `18px ${theme.tokens.padding.speace3}`,
                background: theme.tokens.colors.background2,
            },
            '& tbody td': {
                padding: '18px 24px',
                background: '#282B36',
                borderBottom: '1px solid #383B45',
                position: 'relative',
            },
            '& tbody tr:hover td': {
                background: 'rgba(115,193,250,0.08)',
            },
        },
    }));
    return hoc();
}
