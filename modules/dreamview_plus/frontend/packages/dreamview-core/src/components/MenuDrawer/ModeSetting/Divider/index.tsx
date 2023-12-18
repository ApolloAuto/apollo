import React from 'react';
import useStyle from './useStyle';

function Divider() {
    const { classes } = useStyle();
    return <div className={classes['mode-setting-divider']} />;
}

export default React.memo(Divider);
