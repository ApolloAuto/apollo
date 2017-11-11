import React from "react";
import { observer } from "mobx-react";

@observer
export default class Wheel extends React.Component {
    constructor(props) {
        super(props);
        this.signalColor = {
            off: '#30435E',
            on: '#006AFF',
        };
    }

    render() {
        const { steeringAngle, turnSignal } = this.props;

        const leftArrowColor = (turnSignal === 'LEFT' || turnSignal === 'EMERGENCY')
                               ? this.signalColor.on : this.signalColor.off;
        const rightArrowColor = (turnSignal === 'RIGHT' || turnSignal === 'EMERGENCY')
                                ? this.signalColor.on : this.signalColor.off;

        return (
            <div className="wheel">
                <div className="steerangle-read">{steeringAngle}</div>
                <div className="steerangle-unit">°</div>
                <div className="left-arrow"
                     style={{borderRightColor: leftArrowColor}}></div>
                <div className="right-arrow"
                     style={{borderLeftColor: rightArrowColor}}></div>
                <div className="wheel-background">
                    <div className="wheel-arm"
                         style={{transform: `translate(-50%, -100%) rotate(${steeringAngle}deg`}}/>
                </div>
            </div>
        );
    }
}
