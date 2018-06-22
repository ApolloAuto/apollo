import React from "react";
import { inject, observer } from "mobx-react";

import SETTING from "store/config/ControlGraph.yml";
import ScatterGraph from "components/PNCMonitor/ScatterGraph";

@inject("store") @observer
export default class ControlMonitor extends React.Component {
    generateScatterGraph(name, data) {
        if (SETTING[name] === undefined) {
            console.error("Graph setting not found: ", name);
            return null;
        }

        return (
            <ScatterGraph
                title={SETTING[name].title}
                options={SETTING[name].options}
                properties={SETTING[name].properties}
                data={data} />
        );
    }

    render() {
        const { lastUpdatedTime, data } = this.props.store.controlData;

        if (!lastUpdatedTime) {
            return null;
        }

        return (
            <div>
                {this.generateScatterGraph('trajectoryGraph', data.trajectoryGraph)}
                {this.generateScatterGraph('speedGraph', data.speedGraph)}
                {this.generateScatterGraph('accelerationGraph', data.accelerationGraph)}
                {this.generateScatterGraph('curvatureGraph', data.curvatureGraph)}
                {this.generateScatterGraph('stationErrorGraph', data.stationErrorGraph)}
                {this.generateScatterGraph('lateralErrorGraph', data.lateralErrorGraph)}
                {this.generateScatterGraph('headingErrorGraph', data.headingErrorGraph)}
            </div>
        );
    }
}
