import React from 'react';
import { useMakeStyle } from '@dreamview/dreamview-theme';

function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'dv-app-loading': {
            position: 'fixed',
            zIndex: theme.tokens.zIndex.app,
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            background: 'white',
        },
    }));
    return hoc();
}

function PageLoading() {
    const { classes } = useStyle();
    return <div className={classes['dv-app-loading']}>loading</div>;
}

export default React.memo(PageLoading);
