import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

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
        const { drivingMode, isAutoMode } = this.props;

        // TODO actually implement render "traffic light on".
        return (
            <div className={classNames({
                            "traffic-light-indicator": true,
                            "auto-mode": isAutoMode,
                            "manual-mode": !isAutoMode,
                        })}>
                <Image image={green} className="green" />
                <Image image={yellow} className="yellow" />
                <Image image={red} className="red" />
                <div className="driving-mode">{drivingMode}</div>
            </div>
        );
    }
}
