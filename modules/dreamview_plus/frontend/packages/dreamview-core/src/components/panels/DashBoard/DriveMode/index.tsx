import React from 'react';
import useStyle from './useStyle';

export enum DriveModeEnum {
    MANUAL = 'MANUAL',
    AUTO = 'AUTO',
    DISENGAGED = 'DISENGAGED',
    AUTO_STEER = 'AUTO STEER',
    AUTO_SPEED = 'AUTO SPEED',
    CHASSIS_ERROR = 'CHASSIS ERROR',
    UNKNOWN = 'UNKNOWN',
}

type DriveModeProps = {
    mode: DriveModeEnum;
};

function DriveMode(props: DriveModeProps) {
    const { classes } = useStyle();

    return <div className={classes['dashboard-drive-mode']}>{props.mode}</div>;
}

export default React.memo(DriveMode);
