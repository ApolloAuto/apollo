import React from "react";
import { Tab, Tabs, TabList, TabPanel } from "react-tabs";

import ControlMonitor from "components/PNCMonitor/ControlMonitor";
import LatencyMonitor from "components/PNCMonitor/LatencyMonitor";
import PlanningMonitor from "components/PNCMonitor/PlanningMonitor";
import StoryTellingMonitor from "components/PNCMonitor/StoryTellingMonitor";

export default class PNCMonitor extends React.Component {
    render() {
        return (
            <div className="monitor pnc-monitor">
                <StoryTellingMonitor />
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
