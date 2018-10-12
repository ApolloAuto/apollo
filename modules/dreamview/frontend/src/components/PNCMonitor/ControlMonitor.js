import React from "react";
import { inject, observer } from "mobx-react";

import SETTING from "store/config/ControlGraph.yml";
import ScatterGraph, { generateScatterGraph } from "components/PNCMonitor/ScatterGraph";

@inject("store") @observer
export default class ControlMonitor extends React.Component {
    render() {
        const { lastUpdatedTime, data } = this.props.store.controlData;

        if (!lastUpdatedTime) {
            return null;
        }

        return (
            <div>
                {generateScatterGraph(SETTING.trajectoryGraph, data.trajectoryGraph, {
                    pose: data.pose,
                })}
                {generateScatterGraph(SETTING.speedGraph, data.speedGraph)}
                {generateScatterGraph(SETTING.accelerationGraph, data.accelerationGraph)}
                {generateScatterGraph(SETTING.curvatureGraph, data.curvatureGraph)}
                {generateScatterGraph(SETTING.stationErrorGraph, data.stationErrorGraph)}
                {generateScatterGraph(SETTING.lateralErrorGraph, data.lateralErrorGraph)}
                {generateScatterGraph(SETTING.headingErrorGraph, data.headingErrorGraph)}
            </div>
        );
    }
}
