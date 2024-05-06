import React from 'react';
import useStyle from './useStyle';

interface IRenderDownLoadStatus {
    name: string;
}

function RenderName(props: IRenderDownLoadStatus) {
    const { name } = props;
    const { classes } = useStyle();
    return (
        <div title={name} className={classes['source-name']}>
            {name}
        </div>
    );
}

export default React.memo(RenderName);
