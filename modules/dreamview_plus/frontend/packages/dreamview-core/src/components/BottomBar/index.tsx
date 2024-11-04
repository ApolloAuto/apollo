import React, { useEffect, useState } from 'react';
import { HMIModeOperation, usePickHmiStore, SIM_CONTROL_STATUS } from '@dreamview/dreamview-core/src/store/HmiStore';
import {
    useEventEmitter,
    BusinessEventTypes,
    BusinessEventInfo,
} from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import PlayerControlBar from './PlayerControlBar';
import SimControlBar from './SimControlBar';
import ScenarioBar from './ScenarioBar';
import WayFollowBar from './WayFollowBar';
import AutoDriveBar from './AutoDriveBar';

export default function BottomBar() {
    const [hmi] = usePickHmiStore();

    const [routingInfo, setRoutingInfo] = useState<
        Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>
    >({});

    const EE = useEventEmitter();

    useEffect(() => {
        const onRoutingInfoChange = (value: any) => {
            setRoutingInfo((prev) => ({
                ...prev,
                [value.panelId]: value.routeInfo,
            }));
        };

        EE.on(BusinessEventTypes.SimControlRoute, onRoutingInfoChange);
        return () => {
            EE.off(BusinessEventTypes.SimControlRoute, onRoutingInfoChange);
        };
    }, []);

    useEffect(() => {
        if (hmi.currentScenarioId) {
            setRoutingInfo({});
        }
    }, [hmi.currentScenarioId]);

    if (hmi.currentOperation === HMIModeOperation.PLAY_RECORDER) {
        return <PlayerControlBar routingInfo={routingInfo} />;
    }

    if (hmi.currentOperation === HMIModeOperation.SIM_CONTROL) {
        return <SimControlBar routingInfo={routingInfo} />;
    }

    if (hmi.currentOperation === HMIModeOperation.AUTO_DRIVE) {
        return <AutoDriveBar routingInfo={routingInfo} />;
    }

    if (hmi.currentOperation === HMIModeOperation.SCENARIO) {
        return <ScenarioBar routingInfo={routingInfo} />;
    }

    if (hmi.currentOperation === HMIModeOperation.WAYPOINT_FOLLOW) {
        return <WayFollowBar />;
    }

    return null;
}
