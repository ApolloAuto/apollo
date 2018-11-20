import React from 'react';

import { timestampMsToTimeString } from 'utils/misc';

class PlanningScenarioItem extends React.Component {
    render() {
        const { scenario } = this.props;

        const type = scenario.scenarioType;
        const stage = scenario.stageType ? scenario.stageType.replace(type + '_', '') : '-';

        return (
            <tr className="scenario-history-item">
                <td className="text time">{timestampMsToTimeString(scenario.time, true)}</td>
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
            <div className="scenario-history-container">
                <div className="scenario-history-title">Scenario History</div>
                <table className="scenario-history-table">
                    <tbody>
                        {scenarios.map(scenario => (
                            <PlanningScenarioItem key={`scenario_${scenario.time}`}
                                                  scenario={scenario} />
                        ))}
                    </tbody>
                </table>
            </div>
        );
    }
}
