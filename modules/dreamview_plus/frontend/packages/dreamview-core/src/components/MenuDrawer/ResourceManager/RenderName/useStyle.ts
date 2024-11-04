import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'source-name': {
        position: 'absolute',
        top: 18,
        left: 20,
        right: 20,
        ...theme.util.textEllipsis,
        whiteSpace: 'nowrap',
    },
}));
