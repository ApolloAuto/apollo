import React from "react";
import { inject, observer } from "mobx-react";

import SETTING from "store/config/PlanningGraph.yml";
import ScatterGraph, { generateScatterGraph } from "components/PNCMonitor/ScatterGraph";

@inject("store") @observer
export default class PlanningMonitor extends React.Component {
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
                    boxes={boxes}
                />
            );
        }
        return graphs;
    }

    render() {
        const { planningTime, data } = this.props.store.planningData;

        if (!planningTime) {
            return null;
        }

        return (
            <div>
                {generateScatterGraph(SETTING.speedGraph, data.speedGraph)}
                {generateScatterGraph(SETTING.accelerationGraph, data.accelerationGraph)}
                {generateScatterGraph(SETTING.thetaGraph, data.thetaGraph)}
                {generateScatterGraph(SETTING.kappaGraph, data.kappaGraph)}
                {generateScatterGraph(SETTING.dpPolyGraph, data.dpPolyGraph)}
                {this.generateStGraph(data.stGraph)}
                {generateScatterGraph(
                    SETTING.stSpeedGraph,
                    data.stSpeedGraph.QpSplineStSpeedOptimizer
                )}
                {generateScatterGraph(SETTING.dkappaGraph, data.dkappaGraph)}
            </div>
        );
    }
}
