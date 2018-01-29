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
        const { sequenceNum, data } = this.props.store.controlData;
        const { dimension } = this.props.store;

        if (!sequenceNum) {
            return null;
        }

        return (
            <div>
                {this.generateScatterGraph('trajectoryGraph', data.trajectoryGraph)}
                {this.generateScatterGraph('speedGraph', data.speedGraph)}
                {this.generateScatterGraph('accelerationGraph', data.accelerationGraph)}
                {this.generateScatterGraph('curvatureGraph', data.curvatureGraph)}
            </div>
        );
    }
}
