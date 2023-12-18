import React from 'react';
import useComponentDisplay from '@dreamview/dreamview-core/src/hooks/useComponentDisplay';
import useStyle from './useStyle';

function Background(props: React.PropsWithChildren) {
    const [, { bottomBarHeightString }] = useComponentDisplay();
    const { classes } = useStyle();

    return (
        <div
            style={{ height: `calc(100vh - 176px - ${bottomBarHeightString})` }}
            className={classes['table-background']}
        >
            {props.children}
        </div>
    );
}

export default React.memo(Background);
