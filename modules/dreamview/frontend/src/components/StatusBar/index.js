import React from "react";
import { observer } from "mobx-react";

import AutoMeter from "components/StatusBar/AutoMeter";
import Wheel from "components/StatusBar/Wheel";
import TrafficLightIndicator from "components/StatusBar/TrafficLightIndicator";

@observer
export default class StatusBar extends React.Component {
    render() {
        const { meters } = this.props;

        return (
            <div className="status-bar">
                <AutoMeter throttlePercent={meters.throttlePercent}
                           brakePercent={meters.brakePercent}
                           speed={meters.speed} />
                <Wheel steeringAngle={meters.steeringAngle}
                       turnSignal={meters.turnSignal}
                       drivingMode={meters.drivingMode} />
                <TrafficLightIndicator />
            </div>
        );
    }
}
