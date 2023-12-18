import React, { useMemo } from 'react';
import { apollo } from '@dreamview/dreamview';
import useStyle from './useStyle';

export enum GearPositionEnum {
    P = 'P',
    R = 'R',
    D = 'D',
    N = 'N',
}

type GearProps = {
    gearPosition: apollo.canbus.Chassis.GearPosition;
};

function gearPositionToEnum(position: apollo.canbus.Chassis.GearPosition | string) {
    switch (position) {
        case 'GEAR_PARKING':
            return GearPositionEnum.P;
        case 'GEAR_REVERSE':
            return GearPositionEnum.R;
        case 'GEAR_DRIVE':
            return GearPositionEnum.D;
        default:
            return GearPositionEnum.N;
    }
}

function Gear(props: GearProps) {
    const { classes } = useStyle();

    return (
        <div className={classes['dashboard-gear']}>
            <div
                style={{
                    marginLeft: 12,
                }}
                className={
                    gearPositionToEnum(props?.gearPosition) === GearPositionEnum.P
                        ? classes[('dashboard-gear-position', 'dashboard-gear-position-active')]
                        : classes['dashboard-gear-position']
                }
            >
                P
            </div>
            <div
                className={
                    gearPositionToEnum(props?.gearPosition) === GearPositionEnum.R
                        ? classes[('dashboard-gear-position', 'dashboard-gear-position-active')]
                        : classes['dashboard-gear-position']
                }
            >
                R
            </div>
            <div
                className={
                    gearPositionToEnum(props?.gearPosition) === GearPositionEnum.D
                        ? classes[('dashboard-gear-position', 'dashboard-gear-position-active')]
                        : classes['dashboard-gear-position']
                }
            >
                D
            </div>
            <div
                style={{
                    marginRight: 12,
                }}
                className={
                    gearPositionToEnum(props?.gearPosition) === GearPositionEnum.N
                        ? classes[('dashboard-gear-position', 'dashboard-gear-position-active')]
                        : classes['dashboard-gear-position']
                }
            >
                N
            </div>
        </div>
    );
}

export default React.memo(Gear);
