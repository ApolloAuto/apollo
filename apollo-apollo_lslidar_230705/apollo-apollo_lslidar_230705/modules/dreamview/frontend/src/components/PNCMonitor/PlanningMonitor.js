import React from 'react';
import { inject, observer } from 'mobx-react';
import _ from 'lodash';

import SETTING from 'store/config/PlanningGraph.yml';
import ScatterGraph, { generateScatterGraph } from 'components/PNCMonitor/ScatterGraph';
import PlanningScenarioTable from 'components/PNCMonitor/PlanningScenarioTable';

@inject('store') @observer
export default class PlanningMonitor extends React.Component {
  generateGraphsFromDatasets(settingName, datasets) {
    const setting = SETTING[settingName];
    if (!setting) {
      console.error('No such setting name found in PlanningGraph.yml:', settingName);
      return null;
    }

    return _.get(setting, 'datasets', []).map(({ name, graphTitle }) => {
      const graph = datasets[name];
      const polygons = graph ? graph.obstaclesBoundary : [];
      return (
                <ScatterGraph
                    key={`${settingName}_${name}`}
                    title={graphTitle}
                    options={setting.options}
                    properties={setting.properties}
                    data={{ lines: graph, polygons }}
                />
      );
    });
  }

  render() {
    const {
      planningTimeSec, data, chartData, scenarioHistory,
    } = this.props.store.planningData;

    if (!planningTimeSec) {
      return null;
    }

    const chartCount = {};

    return (
            <div>
                <PlanningScenarioTable scenarios={scenarioHistory} />
                {chartData.map((chart) => {
                  // Adding count to chart key to prevent duplicate chart title
                  if (!chartCount[chart.title]) {
                    chartCount[chart.title] = 1;
                  } else {
                    chartCount[chart.title] += 1;
                  }

                  return (
                        <ScatterGraph
                            key={`custom_${chart.title}_${chartCount[chart.title]}`}
                            title={chart.title}
                            options={chart.options}
                            properties={chart.properties}
                            data={chart.data}
                        />
                  );
                })}
                {generateScatterGraph(SETTING.speedGraph, data.speedGraph)}
                {generateScatterGraph(SETTING.accelerationGraph, data.accelerationGraph)}
                {generateScatterGraph(SETTING.planningThetaGraph, data.thetaGraph)}
                {generateScatterGraph(SETTING.planningKappaGraph, data.kappaGraph)}
                {this.generateGraphsFromDatasets('stGraph', data.stGraph)}
                {this.generateGraphsFromDatasets('stSpeedGraph', data.stSpeedGraph)}
                {generateScatterGraph(SETTING.planningDkappaGraph, data.dkappaGraph)}
                {generateScatterGraph(SETTING.referenceLineThetaGraph, data.thetaGraph)}
                {generateScatterGraph(SETTING.referenceLineKappaGraph, data.kappaGraph)}
                {generateScatterGraph(SETTING.referenceLineDkappaGraph, data.dkappaGraph)}
            </div>
    );
  }
}
