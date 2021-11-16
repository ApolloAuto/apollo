import React from 'react';

import { timestampMsToTimeString } from 'utils/misc';

class PlanningScenarioItem extends React.Component {
  render() {
    const { scenario } = this.props;

    const type = scenario.scenarioType;
    const stage = scenario.stageType ? scenario.stageType.replace(`${type}_`, '') : '-';

    return (
            <tr className="monitor-table-item">
                <td className="text time">{timestampMsToTimeString(scenario.timeSec * 1000, true)}</td>
                <td className="text">{type}</td>
                <td className="text">{stage}</td>
            </tr>
    );
  }
}

export default class PlanningScenarioTable extends React.Component {
  render() {
    const { scenarios } = this.props;

    return (
            <div className="monitor-table-container">
                <div className="monitor-table-title">Scenario History</div>
                <table className="monitor-table">
                    <tbody>
                        {scenarios.map((scenario) => (
                            <PlanningScenarioItem
                                key={`scenario_${scenario.timeSec}`}
                                scenario={scenario}
                            />
                        ))}
                    </tbody>
                </table>
            </div>
    );
  }
}
