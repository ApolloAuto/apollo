import React, { useEffect, useMemo, useState } from 'react';
import { type apollo } from '@dreamview/dreamview';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import Panel from '../base/Panel';
import { StreamDataNames } from '../../../services/api/types';
import DriveMode, { DriveModeEnum } from './DriveMode';
import SignalAndGear from './SignalAndGear';
import SpeedAndRotation from './SpeedAndRotation';
import { usePanelContext } from '../base/store/PanelStore';
import CustomScroll from '../../CustomScroll';

enum DisengageType {
    DISENGAGE_NONE = 'DISENGAGE_NONE',
    DISENGAGE_UNKNOWN = 1,
    DISENGAGE_MANUAL = 'DISENGAGE_MANUAL',
    DISENGAGE_EMERGENCY = 'DISENGAGE_EMERGENCY',
    DISENGAGE_AUTO_STEER_ONLY = 'DISENGAGE_AUTO_STEER_ONLY',
    DISENGAGE_AUTO_SPEED_ONLY = 'DISENGAGE_AUTO_SPEED_ONLY',
    DISENGAGE_CHASSIS_ERROR = 'DISENGAGE_CHASSIS_ERROR',
}

function toDrivingMode(disengageType: DisengageType): DriveModeEnum {
    switch (disengageType) {
        case DisengageType.DISENGAGE_MANUAL:
            return DriveModeEnum.MANUAL;
        case DisengageType.DISENGAGE_NONE:
            return DriveModeEnum.AUTO;
        case DisengageType.DISENGAGE_EMERGENCY:
            return DriveModeEnum.DISENGAGED;
        case DisengageType.DISENGAGE_AUTO_STEER_ONLY:
            return DriveModeEnum.AUTO_STEER;
        case DisengageType.DISENGAGE_AUTO_SPEED_ONLY:
            return DriveModeEnum.AUTO_SPEED;
        case DisengageType.DISENGAGE_CHASSIS_ERROR:
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
    const { initSubscription } = panelContext;
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
        <CustomScroll className={classes['panel-dash-board']}>
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
        </CustomScroll>
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
