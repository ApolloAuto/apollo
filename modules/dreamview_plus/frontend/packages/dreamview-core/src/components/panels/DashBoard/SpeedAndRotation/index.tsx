import React, { useMemo } from 'react';
import useStyle from './useStyle';
import SpeedInfo from './SpeedInfo';
import RotationInfo from './RotationInfo';

type SpeedAndRotationProps = {
    speedInfo?: {
        speed?: number;
        throttlePercent?: number;
        brakePercent?: number;
    };
    rotationInfo?: {
        steeringPercentage?: number;
        steeringAngle?: number;
        turnSignal?: string;
    };
};

function SpeedAndRotation(props: SpeedAndRotationProps) {
    const { classes } = useStyle();

    return (
        <div className={classes['dash-board-speed-rotation']}>
            <SpeedInfo
                speed={props?.speedInfo?.speed}
                throttlePercent={props?.speedInfo?.throttlePercent}
                brakePercent={props?.speedInfo?.brakePercent}
            />
            <RotationInfo
                steeringPercentage={props?.rotationInfo?.steeringPercentage}
                steeringAngle={props?.rotationInfo?.steeringAngle}
                turnSignal={props?.rotationInfo?.turnSignal}
            />
        </div>
    );
}

export default React.memo(SpeedAndRotation);
