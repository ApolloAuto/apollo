import React, { useEffect, useMemo, useState } from 'react';
import { apollo } from '@dreamview/dreamview';
import useStyle from './useStyle';
import Panel from '../base/Panel';
import { StreamDataNames } from '../../../services/api/types';
import DriveMode, { DriveModeEnum } from './DriveMode';
import SignalAndGear from './SignalAndGear';
import SpeedAndRotation from './SpeedAndRotation';
import { usePanelContext } from '../base/store/PanelStore';
import ChannelSelectFactory from '../base/ChannelSelect/ChannelSelectFactory';
import DemoChannelSelect from '../base/ChannelSelect/demo';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;
type DisengageType = apollo.dreamview.Object.DisengageType;

function toDrivingMode(disengageType: DisengageType): DriveModeEnum {
    switch (disengageType) {
        case apollo.dreamview.Object.DisengageType.DISENGAGE_MANUAL:
            return DriveModeEnum.MANUAL;
        case apollo.dreamview.Object.DisengageType.DISENGAGE_NONE:
            return DriveModeEnum.AUTO;
        case apollo.dreamview.Object.DisengageType.DISENGAGE_EMERGENCY:
            return DriveModeEnum.DISENGAGED;
        case apollo.dreamview.Object.DisengageType.DISENGAGE_AUTO_STEER_ONLY:
            return DriveModeEnum.AUTO_STEER;
        case apollo.dreamview.Object.DisengageType.DISENGAGE_AUTO_SPEED_ONLY:
            return DriveModeEnum.AUTO_SPEED;
        case apollo.dreamview.Object.DisengageType.DISENGAGE_CHASSIS_ERROR:
            return DriveModeEnum.CHASSIS_ERROR;
        default:
            return DriveModeEnum.UNKNOWN;
    }
}

type DashBoardState = {
    autoDrivingCar?: apollo.dreamview.IObject | null;
    trafficSignal?: apollo.dreamview.IObject | null;
};

function InnerDashBoard() {
    const panelContext = usePanelContext();
    const { logger, panelId, initSubscription, registerFullScreenHooks } = panelContext;
    const [rawState, setRawState] = useState<DashBoardState>({});
    const dashBoardState = useMemo(
        () => ({
            autoDrivingCar: rawState?.autoDrivingCar,
            trafficSignal: rawState?.trafficSignal,
        }),
        [rawState],
    );

    const { classes } = useStyle();

    useEffect(() => {
        initSubscription({
            [StreamDataNames.SIM_WORLD]: {
                consumer: (data) => {
                    setRawState(data);
                },
            },
        });
    }, []);

    return (
        <div className={classes['panel-dash-board']}>
            <DriveMode mode={toDrivingMode(dashBoardState?.autoDrivingCar?.disengageType)} />
            <SignalAndGear
                color={dashBoardState?.trafficSignal?.currentSignal}
                gearPosition={dashBoardState?.autoDrivingCar?.gearLocation}
            />
            <SpeedAndRotation
                speedInfo={{
                    speed: dashBoardState?.autoDrivingCar?.speed,
                    throttlePercent: dashBoardState?.autoDrivingCar?.throttlePercentage,
                    brakePercent: dashBoardState?.autoDrivingCar?.brakePercentage,
                }}
                rotationInfo={{
                    steeringPercentage: dashBoardState?.autoDrivingCar?.steeringPercentage,
                    steeringAngle: dashBoardState?.autoDrivingCar?.steeringAngle,
                    turnSignal: dashBoardState?.autoDrivingCar?.currentSignal,
                }}
            />
        </div>
    );
}

InnerDashBoard.displayName = 'DashBoard';

export function DashBoardOrigin(props: any) {
    const C = useMemo(
        () =>
            Panel({
                PanelComponent: InnerDashBoard,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.SIM_WORLD, needChannel: false }],
            }),
        [],
    );

    return <C {...props} />;
}

export const DashBoard = React.memo(DashBoardOrigin);
