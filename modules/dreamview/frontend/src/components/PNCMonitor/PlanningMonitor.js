import React from "react";
import { inject, observer } from "mobx-react";

import SETTING from "store/config/PlanningGraph.yml";
import ScatterGraph, { generateScatterGraph } from "components/PNCMonitor/ScatterGraph";
import PlanningScenarioTable from "components/PNCMonitor/PlanningScenarioTable";
import { timestampMsToTimeString } from "utils/misc";

@inject("store") @observer
export default class PlanningMonitor extends React.Component {
    generateStGraph(stGraph) {
        const graphs = [];
        const displayNames = {
            'DpStSpeedOptimizer': 'Speed Heuristic',
            'PiecewiseJerkSpeedOptimizer': 'Planning S-T Graph',
        };
        for (const pathName in displayNames) {
            const graph = stGraph[pathName];
            const polygons = graph ? graph.obstaclesBoundary : [];
            graphs.push(
                <ScatterGraph
                    key={'stGraph_' + pathName}
                    title={displayNames[pathName]}
                    options={SETTING.stGraph.options}
                    properties={SETTING.stGraph.properties}
                    data={{ lines: graph, polygons: polygons }}
                />
            );
        }
        return graphs;
    }

    render() {
        const { planningTime, data, chartData, scenarioHistory } = this.props.store.planningData;

        if (!planningTime) {
            return null;
        }

        return (
            <div>
                <PlanningScenarioTable scenarios={scenarioHistory} />
                {chartData.map(chart => {
                    return <ScatterGraph key={chart.title}
                                         title={chart.title}
                                         options={chart.options}
                                         properties={chart.properties}
                                         data={chart.data} />;
                })}
                {generateScatterGraph(SETTING.speedGraph, data.speedGraph)}
                {generateScatterGraph(SETTING.accelerationGraph, data.accelerationGraph)}
                {generateScatterGraph(SETTING.planningThetaGraph, data.thetaGraph)}
                {generateScatterGraph(SETTING.planningKappaGraph, data.kappaGraph)}
                {this.generateStGraph(data.stGraph)}
                {generateScatterGraph(
                    SETTING.stSpeedGraph,
                    data.stSpeedGraph.PiecewiseJerkSpeedOptimizer
                )}
                {generateScatterGraph(SETTING.planningDkappaGraph, data.dkappaGraph)}
                {generateScatterGraph(SETTING.referenceLineThetaGraph, data.thetaGraph)}
                {generateScatterGraph(SETTING.referenceLineKappaGraph, data.kappaGraph)}
                {generateScatterGraph(SETTING.referenceLineDkappaGraph, data.dkappaGraph)}
            </div>
        );
    }
}
