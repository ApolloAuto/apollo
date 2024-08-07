import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'table-background': {
        overflow: 'hidden',
        borderRadius: '10px',
        background: theme.components.table.bodyBgColor,
    },
}));
