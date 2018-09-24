import React from "react";
import { inject, observer } from "mobx-react";

import setting from "store/config/LatencyGraph.yml";
import ScatterGraph, { generateScatterGraph } from "components/PNCMonitor/ScatterGraph";

@inject("store") @observer
export default class LatencyMonitor extends React.Component {
    render() {
        const { lastUpdatedTime, data } = this.props.store.latency;

        if (!data) {
            return null;
        }

        const graphs = {};
        Object.keys(data).forEach(moduleName => {
            graphs[moduleName] = data[moduleName];
        });
        return generateScatterGraph(setting, graphs);
    }
}
