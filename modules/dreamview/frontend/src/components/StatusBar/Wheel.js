import React from "react";
import { observer } from "mobx-react";

import Image from "components/common/Image";
import wheelImage from "assets/images/wheel/wheel.png";
import wheelBackground from "assets/images/wheel/wheel_background.png";
import leftArrowOff from "assets/images/wheel/left_off";
import rightArrowOff from "assets/images/wheel/right_off";

@observer
export default class Wheel extends React.Component {
    render() {
        const { steeringAngle } = this.props;

        return (
            <div className="wheel">
                <Image image={wheelBackground}
                       className="wheel-background" />
                <Image image={wheelImage}
                       style={{transform: `rotate(${steeringAngle}deg)`}}
                       className="wheel-image" />
                <Image image={leftArrowOff}
                       className="left-arrow"/>
                <Image image={rightArrowOff}
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
