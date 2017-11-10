import React from "react";
import { observer } from "mobx-react";

import AutoMeter from "components/StatusBar/AutoMeter";
import Notification from "components/StatusBar/Notification";
import TrafficLightIndicator from "components/StatusBar/TrafficLightIndicator";
import Wheel from "components/StatusBar/Wheel";


@observer
export default class StatusBar extends React.Component {
    render() {
        const { meters, showNotification, monitor } = this.props;

        return (
            <div className="status-bar">
                {showNotification && <Notification monitor={monitor} />}
                <AutoMeter throttlePercent={meters.throttlePercent}
                           brakePercent={meters.brakePercent}
                           speed={meters.speed} />
                <Wheel steeringAngle={meters.steeringAngle}
                       turnSignal={meters.turnSignal} />
                <TrafficLightIndicator
                       drivingMode={meters.drivingMode}
                       isAutoMode={meters.isAutoMode}/>
            </div>
        );
    }
}
