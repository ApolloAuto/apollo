import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

const ColorCodeMapping = {
    'GREEN': 'rgba(79, 198, 105, 0.8)',
    'YELLOW': 'rgba(239, 255, 0, 0.8)',
    'RED': 'rgba(180, 49, 49, 0.8)',
    'BLACK': 'rgba(42, 50, 56, 0.8)',
    'UNKNOWN': 'rgba(42, 50, 56, 0.8)',
    '': 'rgba(42, 50, 56, 0.8)',
};

class TrafficLight extends React.Component {
    render() {
        const { color } = this.props;

        return (
            <svg className="traffic-light" viewBox="0 0 30 30" height="28" width="28">
                <circle cx="15" cy="15" r="15" fill={ColorCodeMapping[color]} />
            </svg>
        );
    }
}

@observer
export default class TrafficLightIndicator extends React.Component {
    render() {
        const { trafficLightColor, drivingMode, isAutoMode } = this.props;

        return (
            <div className={classNames({
                            "traffic-light-indicator": true,
                            "auto-mode": isAutoMode,
                            "manual-mode": !isAutoMode,
                        })}>
                <TrafficLight color={trafficLightColor}/>
                <div className="driving-mode">{drivingMode}</div>
            </div>
        );
    }
}
