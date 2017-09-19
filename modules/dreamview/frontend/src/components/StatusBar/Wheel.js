import React from "react";
import { observer } from "mobx-react";

import Image from "components/common/Image";
import wheelImage from "assets/images/wheel/wheel.png";
import wheelBackground from "assets/images/wheel/wheel_background.png";
import leftArrowOff from "assets/images/wheel/left_off";
import rightArrowOff from "assets/images/wheel/right_off";
import leftArrowOn from "assets/images/wheel/left_on";
import rightArrowOn from "assets/images/wheel/right_on";

@observer
export default class Wheel extends React.Component {
    render() {
        const { steeringAngle, turnSignal } = this.props;

        const leftArrowSrc = (turnSignal === 'LEFT' || turnSignal === 'EMERGENCY')
                             ? leftArrowOn : leftArrowOff;
        const rightArrowSrc = (turnSignal === 'RIGHT' || turnSignal === 'EMERGENCY')
                              ? rightArrowOn : rightArrowOff;

        return (
            <div className="wheel">
                <Image image={wheelBackground}
                       className="wheel-background" />
                <Image image={wheelImage}
                       style={{transform: `rotate(${steeringAngle}deg)`}}
                       className="wheel-image" />
                <Image image={leftArrowSrc}
                       className="left-arrow"/>
                <Image image={rightArrowSrc}
                       className="right-arrow"/>
                <span className="left-label-text label-text">
                    Left
                </span>
                <span className="right-label-text label-text">
                    Right
                </span>
            </div>
        );
    }
}
