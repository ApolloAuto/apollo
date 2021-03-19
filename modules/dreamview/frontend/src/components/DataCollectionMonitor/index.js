import React from 'react';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';
import { inject, observer } from 'mobx-react';

import WS from 'store/websocket';
import ScenarioCollectionMonitor from 'components/DataCollectionMonitor/ScenarioCollectionMonitor';
import { MonitorItem } from '../Tasks/Console';
import { timestampMsToTimeString } from 'utils/misc';

@inject('store') @observer
export default class DataCollectionMonitor extends React.Component {
  constructor(props) {
    super(props);

    this.handlePreprocess = this.handlePreprocess.bind(this);
  }

  handlePreprocess() {
    const hmi = this.props.store.hmi;
    if (!hmi.preprocessIsRunning) {
      WS.startPreprocessData([], 'VehicleCalibrationPreprocess');
      hmi.preprocessStarted = true;
      hmi.unexpectedAborted = false;
    }
  }

  render() {
    const hmi = this.props.store.hmi;
    const tabs = [];
    const tabPanels = [];
    hmi.dataCollectionProgress.entries().forEach(([scenarioName, categories]) => {
      tabs.push(<Tab key={scenarioName}>{scenarioName}</Tab>);

      tabPanels.push(
        <TabPanel key={scenarioName}>
          <ScenarioCollectionMonitor
            statusMap={hmi.dataCollectionUpdateStatus.get(scenarioName)}
            progressMap={categories}
            startUpdateProgress={hmi.startUpdateDataCollectionProgress}
          />
        </TabPanel>
      );
    });

    return (
      <div className="monitor data-collection-monitor">
            <div className="monitor data-collection-monitor vehicle-calibration-panel">
                <Tabs>
                    <TabList>{tabs}</TabList>
                    {tabPanels}
                </Tabs>
        </div>
        <div className="preprocess-bar category">
          <div className="category-description">
            <button
              className="preprocess-btn"
              disabled={!hmi.canStartDataCollectionPreprocess}
              onClick={this.handlePreprocess}
            >
              Preprocess
        </button>
          </div>
          <div className="category-progress-background">
            <span
              className={true
                ? 'category-completed' : 'category-in-progress'}
              style={{ width: `${hmi.preprocessProgress}%` }}
            />
          </div>
        </div>
        <div className="preprocess-msg">
          <ul className="preprocess-console">
            <MonitorItem
              text={hmi.logString}
              level={hmi.preprocessStatus}
              time={hmi.preprocessMonitorItemTimeStamp
                ? timestampMsToTimeString(hmi.preprocessMonitorItemTimeStamp)
                : hmi.preprocessMonitorItemTimeStamp}
            >
            </MonitorItem>
          </ul>
          </div>
        </div>
    );
  }
}
