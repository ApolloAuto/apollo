import React from "react";
import { observer } from "mobx-react";

import Image from "components/common/Image";

import throttleImage0 from "assets/images/throttles/A-0p.png";
import throttleImage10 from "assets/images/throttles/A-10p.png";
import throttleImage20 from "assets/images/throttles/A-20p.png";
import throttleImage30 from "assets/images/throttles/A-30p.png";
import throttleImage40 from "assets/images/throttles/A-40p.png";
import throttleImage50 from "assets/images/throttles/A-50p.png";
import throttleImage60 from "assets/images/throttles/A-60p.png";
import throttleImage70 from "assets/images/throttles/A-70p.png";
import throttleImage80 from "assets/images/throttles/A-80p.png";
import throttleImage90 from "assets/images/throttles/A-90p.png";
import throttleImage100 from "assets/images/throttles/A-100p.png";

import brakeImage0 from "assets/images/brakes/B-0p.png";
import brakeImage10 from "assets/images/brakes/B-10p.png";
import brakeImage20 from "assets/images/brakes/B-20p.png";
import brakeImage30 from "assets/images/brakes/B-30p.png";
import brakeImage40 from "assets/images/brakes/B-40p.png";
import brakeImage50 from "assets/images/brakes/B-50p.png";
import brakeImage60 from "assets/images/brakes/B-60p.png";
import brakeImage70 from "assets/images/brakes/B-70p.png";
import brakeImage80 from "assets/images/brakes/B-80p.png";
import brakeImage90 from "assets/images/brakes/B-90p.png";
import brakeImage100 from "assets/images/brakes/B-100p.png";


const THROTTLES = [
    throttleImage0,
    throttleImage10,
    throttleImage20,
    throttleImage30,
    throttleImage40,
    throttleImage50,
    throttleImage60,
    throttleImage70,
    throttleImage80,
    throttleImage90,
    throttleImage100,
];

const BRAKES = [
    brakeImage0,
    brakeImage10,
    brakeImage20,
    brakeImage30,
    brakeImage40,
    brakeImage50,
    brakeImage60,
    brakeImage70,
    brakeImage80,
    brakeImage90,
    brakeImage100,
];


@observer
export default class AutoMeter extends React.Component {
    render() {
        const { throttlePercent, brakePercent, speed,
                drivingMode } = this.props;

        return (
            <div>
                <Image image={THROTTLES[throttlePercent / 10]}
                       className="throttle-panel" />
                <Image image={BRAKES[brakePercent / 10]}
                       className="brake-panel" />
                <span className="speed-unit">
                    km/h
                </span>
                <div className="speed-panel">
                    <span className="speed-read">
                        {speed}
                    </span>
                    <div className="driving-mode">
                        {drivingMode}
                    </div>
                </div>
                <span className="meter-text-accelerator label-text">
                    Accelerator
                </span>
                <span className="meter-text-brake label-text">
                    Brake
                </span>
            </div>
        );
    }
}
