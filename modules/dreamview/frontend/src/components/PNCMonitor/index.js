import React from "react";
import { inject, observer } from "mobx-react";

import SETTING from "store/config/graph.yml";
import ScatterGraph from "components/PNCMonitor/ScatterGraph";

@inject("store") @observer
export default class PNCMonitor extends React.Component {
    generateStGraph(stGraph) {
        const graphs = [];
        const names = ['DpStSpeedOptimizer', 'QpSplineStSpeedOptimizer'];
        for (const name of names) {
            const graph = stGraph[name];
            const boxes = graph ? graph.obstaclesBoundary : [];
            graphs.push(
                <ScatterGraph
                     key={'stGraph_' + name}
                     title={name}
                     options={SETTING.stGraph.options}
                     properties={SETTING.stGraph.properties}
                     data={graph}
                     boxes={boxes} />
            );
        }
        return graphs;
    }

    render() {
        const { sequenceNum, data } = this.props.store.planning;
        const { dimension } = this.props.store;

        if (!sequenceNum) {
            return null;
        }

        return (
            <div className="pnc-monitor">
                <ScatterGraph
                        title={SETTING.slGraph.title}
                        options={SETTING.slGraph.options}
                        properties={SETTING.slGraph.properties}
                        data={data.slGraph} />
                <ScatterGraph
                        title={SETTING.stSpeedGraph.title}
                        options={SETTING.stSpeedGraph.options}
                        properties={SETTING.stSpeedGraph.properties}
                        data={data.stSpeedGraph.QpSplineStSpeedOptimizer} />
                <ScatterGraph
                        title={SETTING.speedGraph.title}
                        options={SETTING.speedGraph.options}
                        properties={SETTING.speedGraph.properties}
                        data={data.speedGraph} />
                {this.generateStGraph(data.stGraph)}
            </div>
        );
    }
}
