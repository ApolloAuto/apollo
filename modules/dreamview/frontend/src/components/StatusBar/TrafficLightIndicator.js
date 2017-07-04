import React from "react";
import { observer } from "mobx-react";

import Image from "components/common/Image";
import yellow from
"assets/images/traffic_light_indicator/yellow_background.png";
import green from
"assets/images/traffic_light_indicator/green_background.png";
import red from
"assets/images/traffic_light_indicator/red_background.png";

@observer
export default class TrafficLightIndicator extends React.Component {
    render() {
        // TODO actually implement render "traffic light on".
        return (
            <div className="traffic-light-indicator">
                <Image image={green} className="green" />
                <Image image={yellow} className="yellow" />
                <Image image={red} className="red" />
                <span className="traffic-light-label-text label-text">
                    Traffic Signal
                </span>
            </div>
        );
    }
}
