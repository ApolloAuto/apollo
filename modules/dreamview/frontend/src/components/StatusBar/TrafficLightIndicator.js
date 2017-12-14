import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

const ColorCodeMapping = {
    'GREEN': 'rgba(79, 198, 105, 0.8)',
    'YELLOW': 'rgba(239, 255, 0, 0.8)',
    'RED': 'rgba(180, 49, 49, 0.8)',
    'BLACK': 'rgba(30, 30, 30, 0.8)',
    'UNKNOWN': 'rgba(30, 30, 30, 0.8)',
    '': 'rgba(30, 30, 30, 0.8)',
};

class TrafficLight extends React.Component {
    render() {
        const { color, text } = this.props;

        return (
            <div className="traffic-light" >
                <svg className="symbol" viewBox="0 0 30 30" height="28" width="28">
                    <circle cx="15" cy="15" r="15" fill={color} />
                </svg>
                <div className="text">{text}</div>
            </div>
        );
    }
}

@observer
export default class TrafficLightIndicator extends React.Component {
    render() {
        const { trafficLightColor, drivingMode, isAutoMode } = this.props;

        const trafficLightText = trafficLightColor === '' ? "NO SIGNAL" : trafficLightColor;

        return (
            <div className="traffic-light-indicator">
                <TrafficLight color={ColorCodeMapping[trafficLightColor]} text={trafficLightText}/>
                <div className={classNames({
                            "driving-mode": true,
                            "auto-mode": isAutoMode,
                            "manual-mode": !isAutoMode,
                        })}><span className="text">{drivingMode}</span></div>
            </div>
        );
    }
}
