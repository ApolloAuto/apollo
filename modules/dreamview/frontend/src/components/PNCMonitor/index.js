import React from "react";

import PlanningMonitor from "components/PNCMonitor/PlanningMonitor";
import ControlMonitor from "components/PNCMonitor/ControlMonitor";

export default class PNCMonitor extends React.Component {

    render() {
        return (
            <div className="pnc-monitor">
                <PlanningMonitor />
                <ControlMonitor />
            </div>
        );
    }
}

