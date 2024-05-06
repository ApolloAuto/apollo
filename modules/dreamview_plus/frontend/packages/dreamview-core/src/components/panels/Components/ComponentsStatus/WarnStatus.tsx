import { IconIcWarningMessage } from '@dreamview/dreamview-ui';
import React from 'react';
import useStyle from './useStyle';

export function WarnStatus() {
    const { classes } = useStyle();

    return (
        <div className={classes['status-warn']}>
            <IconIcWarningMessage
                style={{
                    fontSize: '16px',
                    marginRight: '6px',
                }}
            />
            WARN
        </div>
    );
}
