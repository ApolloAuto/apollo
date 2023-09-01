import React from 'react';
import { IconIcAmplification, IconIcReduce } from '@dreamview/dreamview-ui';
import useStyle from '../useStyle';

export default function ViewMenu(props: any) {
    const { carviz } = props;
    const { classes } = useStyle();
    return (
        <div className={classes['view-menu-btn-container']}>
            <div className={classes['view-menu-btn-item']} onClick={() => carviz.view?.updateViewDistance(-10)}>
                <IconIcAmplification style={{ fontSize: '16px', color: '#96A5C1' }} />
            </div>
            <div className={classes['view-menu-btn-item']} onClick={() => carviz.view?.updateViewDistance(10)}>
                <IconIcReduce style={{ fontSize: '16px', color: '#96A5C1' }} />
            </div>
        </div>
    );
}
