import React from "react";
import { inject, observer } from "mobx-react";

import SETTING from "store/config/PlanningGraph.yml";
import ScatterGraph from "components/PNCMonitor/ScatterGraph";

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
                     boxes={boxes} />
            );
        }
        return graphs;
    }

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
        const { planningTime, data, latencyGraph } = this.props.store.planningData;

        if (!planningTime) {
            return null;
        }

        return (
            <div>
                {this.generateScatterGraph('speedGraph', data.speedGraph)}
                {this.generateScatterGraph('accelerationGraph', data.accelerationGraph)}
                {this.generateScatterGraph('thetaGraph', data.thetaGraph)}
                {this.generateScatterGraph('kappaGraph', data.kappaGraph)}
                {this.generateScatterGraph('dpPolyGraph', data.dpPolyGraph)}
                {this.generateStGraph(data.stGraph)}
                {this.generateScatterGraph('stSpeedGraph',
                        data.stSpeedGraph.QpSplineStSpeedOptimizer)}
                {this.generateScatterGraph('latencyGraph', latencyGraph)}
                {this.generateScatterGraph('dkappaGraph', data.dkappaGraph)}
            </div>
        );
    }
}
