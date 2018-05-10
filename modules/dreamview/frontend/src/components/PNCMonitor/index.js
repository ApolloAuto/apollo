import React from "react";
import { Tab, Tabs, TabList, TabPanel } from 'react-tabs';

import PlanningMonitor from "components/PNCMonitor/PlanningMonitor";
import ControlMonitor from "components/PNCMonitor/ControlMonitor";

export default class PNCMonitor extends React.Component {

    render() {
        return (
            <div className="pnc-monitor">
                <Tabs>
                    <TabList>
                        <Tab>Planning</Tab>
                        <Tab>Control</Tab>
                    </TabList>
                    <TabPanel>
                        <PlanningMonitor />
                    </TabPanel>
                    <TabPanel>
                        <ControlMonitor />
                    </TabPanel>
                </Tabs>
            </div>
        );
    }
}

