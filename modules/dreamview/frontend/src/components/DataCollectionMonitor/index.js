import React from "react";
import { Tab, Tabs, TabList, TabPanel } from "react-tabs";
import { observer } from "mobx-react";

import ScenarioCollectionMonitor from "components/DataCollectionMonitor/ScenarioCollectionMonitor";

@observer
export default class DataCollectionMonitor extends React.Component {
    render() {
        const { dataCollectionUpdateStatus, dataCollectionProgress } = this.props;

        if (!dataCollectionProgress || dataCollectionUpdateStatus.size === 0) {
            return <div className="no-data">No Data Found</div>;
        }

        const tabs = [];
        const tabPanels = [];
        dataCollectionProgress.entries().forEach(([scenarioName, categories]) => {
            tabs.push(<Tab key={scenarioName}>{scenarioName}</Tab>);

            tabPanels.push(
                <TabPanel key={scenarioName}>
                    <ScenarioCollectionMonitor
                        statusMap={dataCollectionUpdateStatus.get(scenarioName)}
                        progressMap={categories} />
                </TabPanel>
            );
        });

        return (
            <div className="monitor data-collection-monitor">
                <Tabs>
                    <TabList>{tabs}</TabList>
                    {tabPanels}
                </Tabs>
            </div>
        );
    }
}
