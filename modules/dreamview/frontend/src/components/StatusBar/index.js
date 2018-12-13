import React from "react";
import { observer } from "mobx-react";

import AutoMeter from "components/StatusBar/AutoMeter";
import Notification from "components/StatusBar/Notification";
import TrafficLightIndicator from "components/StatusBar/TrafficLightIndicator";
import DrivingMode from "components/StatusBar/DrivingMode";
import Wheel from "components/StatusBar/Wheel";
import Rss from "components/StatusBar/Rss";


@observer
export default class StatusBar extends React.Component {
    render() {
        const { meters, trafficSignal, showNotification,
                showPlanningRSSInfo, monitor } = this.props;

        return (
            <div className="status-bar">
                {showNotification &&
                    <Notification monitor={monitor}
                        showPlanningRSSInfo={showPlanningRSSInfo} />}
                {showPlanningRSSInfo && <Rss monitor={monitor} />}
                <AutoMeter throttlePercent={meters.throttlePercent}
                    brakePercent={meters.brakePercent}
                    speed={meters.speed} />
                <Wheel steeringPercentage={meters.steeringPercentage}
                    steeringAngle={meters.steeringAngle}
                    turnSignal={meters.turnSignal} />
                <div className="traffic-light-and-driving-mode">
                    <TrafficLightIndicator colorName={trafficSignal.color} />
                    <DrivingMode drivingMode={meters.drivingMode}
                        isAutoMode={meters.isAutoMode} />
                </div>
            </div>
        );
    }
}
