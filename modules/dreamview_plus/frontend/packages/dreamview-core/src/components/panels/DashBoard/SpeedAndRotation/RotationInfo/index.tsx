import React, { useMemo } from 'react';
import { IconIcAimingCircle, IconIcDextroversionHighlight, IconIcLeftTurnDefault } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';

type RotationInfoProps = {
    steeringPercentage?: number;
    steeringAngle?: number;
    turnSignal?: string;
};

function RotationInfo(props: RotationInfoProps) {
    const { classes } = useStyle();

    const isLeftOn = useMemo(
        () => props?.turnSignal === 'LEFT' || props?.turnSignal === 'EMERGENCY',
        [props?.turnSignal],
    );
    const isRightOn = useMemo(
        () => props?.turnSignal === 'RIGHT' || props?.turnSignal === 'EMERGENCY',
        [props?.turnSignal],
    );

    const steeringPercentage = useMemo(() => {
        if (props?.steeringPercentage !== undefined && !Number.isNaN(props?.steeringPercentage)) {
            return Math.round(props?.steeringPercentage);
        }

        return 0;
    }, [props?.steeringPercentage]);

    const steeringAngle = useMemo(() => {
        if (props?.steeringAngle !== undefined && !Number.isNaN(props?.steeringAngle)) {
            // eslint-disable-next-line no-unsafe-optional-chaining
            return -Math.round((props?.steeringAngle * 180.0) / Math.PI);
        }

        return 0;
    }, [props?.steeringAngle]);

    return (
        <div className={classes['dashboard-rotation-info']}>
            <div
                style={{
                    marginTop: 10,
                    marginBottom: 10,
                }}
            >
                <div className={classes['dashboard-rotation-info-font']}>{`${steeringPercentage}%`}</div>
                <span className={classes['dashboard-rotation-info-font-small']}>Rotation Ratio</span>
            </div>
            <div className={classes['dashboard-rotation-info-wheel']}>
                <IconIcLeftTurnDefault
                    style={{
                        color: isLeftOn ? '#3288FA' : undefined,
                        fontSize: 16,
                    }}
                />
                {/* <IconIcAimingCircle
                    rotate={steeringAngle}
                    style={{
                        marginLeft: 10,
                        marginRight: 10,
                        fontSize: 60,
                        color: 'linear-gradient(pink, blue)',
                    }}
                /> */}
                <img
                    style={{
                        marginLeft: 10,
                        marginRight: 10,
                        width: 60,
                        height: 60,
                        transform: `rotate(${steeringAngle}deg)`,
                    }}
                    // eslint-disable-next-line global-require
                    src={require('./ic_aiming_circle@3x.png')}
                    alt=''
                />
                <IconIcDextroversionHighlight
                    style={{
                        color: isRightOn ? '#3288FA' : undefined,
                        fontSize: 16,
                    }}
                />
            </div>
        </div>
    );
}

export default React.memo(RotationInfo);
