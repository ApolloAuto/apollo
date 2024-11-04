import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'profile-table': {
        color: theme.components.tab.color,
        '& table': {
            width: '100%',
            borderCollapse: 'separate',
            borderSpacing: 0,
        },
        '& thead': {
            borderRadius: '10px 10px 0 0',
            textAlign: 'left',
            '& th': {
                borderBottom: theme.components.table.headBorderColor,
            },
        },
        '& thead th': {
            padding: `18px ${theme.tokens.padding.speace3}`,
            background: theme.components.table.headBgColor,
        },
        '& tbody td': {
            padding: '18px 24px',
            background: theme.components.table.bodyBgColor,
            borderBottom: theme.components.table.borderBottom,
            position: 'relative',
        },
        '& tbody tr:hover td': {
            background: theme.components.table.tdHoverColor,
        },
    },
}));
