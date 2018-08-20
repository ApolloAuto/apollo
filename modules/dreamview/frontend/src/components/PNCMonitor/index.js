import React from "react";
import { Tab, Tabs, TabList, TabPanel } from "react-tabs";

import PlanningMonitor from "components/PNCMonitor/PlanningMonitor";
import ControlMonitor from "components/PNCMonitor/ControlMonitor";
import LatencyMonitor from "components/PNCMonitor/LatencyMonitor";

export default class PNCMonitor extends React.Component {
    render() {
        return (
            <div className="pnc-monitor">
                <Tabs>
                    <TabList>
                        <Tab>Planning</Tab>
                        <Tab>Control</Tab>
                        <Tab>Latency</Tab>
                    </TabList>
                    <TabPanel>
                        <PlanningMonitor />
                    </TabPanel>
                    <TabPanel>
                        <ControlMonitor />
                    </TabPanel>
                    <TabPanel>
                        <LatencyMonitor />
                    </TabPanel>
                </Tabs>
            </div>
        );
    }
}
