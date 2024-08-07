import { IconPark } from '@dreamview/dreamview-ui';
import React from 'react';
import useStyle from './useStyle';

export function ErrorStatus() {
    const { classes } = useStyle();

    return (
        <div className={classes['status-fatal']}>
            <IconPark
                name='IcErrorMessage'
                style={{
                    fontSize: '16px',
                    marginRight: '6px',
                }}
            />
            ERROR
        </div>
    );
}
