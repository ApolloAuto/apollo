import React, { useMemo } from 'react';
import { apollo } from '@dreamview/dreamview';
import useStyle from './useStyle';
import Signal, { SignalEnum } from './Signal';
import Gear, { GearPositionEnum } from './Gear';

type SignalAndGearProps = {
    color?: string;
    gearPosition?: apollo.canbus.Chassis.GearPosition;
};

function SignalAndGear(props: SignalAndGearProps) {
    const { classes } = useStyle();

    return (
        <div className={classes['dash-board-signal-gear']}>
            <Signal signal={props?.color as SignalEnum} />
            <Gear gearPosition={props?.gearPosition} />
        </div>
    );
}

export default React.memo(SignalAndGear);
