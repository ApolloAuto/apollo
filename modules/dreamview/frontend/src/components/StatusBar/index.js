import React from "react";
import { observer } from "mobx-react";

import AutoMeter from "components/StatusBar/AutoMeter";
import Wheel from "components/StatusBar/Wheel";
import TrafficLightIndicator from "components/StatusBar/TrafficLightIndicator";

class StatusBackground extends React.Component {
    render() {
        const boundary = (
            "M 0 0 H 100 V 100 H 60 C 52 100, 53 90, 52 85 L 48 55 "
            + "C 47 45, 42 40, 40 40 H 0 L 0 0");
        return (
            <div className="status-background">
                <svg width="100%" height="100%" viewBox="0 0 100 100"
                     preserveAspectRatio="none">
                    <path d={boundary} fillOpacity="0.25" />
                </svg>
            </div>
        );
    }
}

@observer
export default class StatusBar extends React.Component {
    render() {
        const { meters } = this.props;

        return (
            <div className="status-bar">
                <StatusBackground />
                <AutoMeter throttlePercent={meters.throttlePercent}
                           brakePercent={meters.brakePercent}
                           speed={meters.speed}
                           drivingMode={meters.drivingMode} />
                <Wheel steeringAngle={meters.steeringAngle}
                       turnSignal={meters.turnSignal} />
                <TrafficLightIndicator />
            </div>
        );
    }
}
