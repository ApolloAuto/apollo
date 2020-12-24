import React from 'react';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';
import { inject, observer } from 'mobx-react';

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
    //多次点击
    const hmi = this.props.store.hmi;
    if (hmi.canStartPreProcess) {
      hmi.canStartPreProcess = false;
      WS.startPreProcessData([], 'VehicleCalibrationPreprocess');
    }
  }

  render() {
    const { dataCollectionUpdateStatus, dataCollectionProgress,
      canStartDataCollectionPreProcess } = this.props;
    const hmi = this.props.store.hmi;

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
                        progressMap={categories}
                    />
                </TabPanel>,
      );
    });

    return (
      <div className="monitor">
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
              disabled={!canStartDataCollectionPreProcess}
              onClick={this.handlePreprocess}
            >
              Preprocess
        </button>
          </div>
          <div className="category-progress-background">
            <span
              className={true
                ? 'category-completed' : 'category-in-progress'}
              style={{ width: `${hmi.preProcessProgress}%` }}
            />
          </div>
        </div>
        <div className="preprocess-msg">
          <ul className="preprocess-console">
            <MonitorItem
              text={hmi.logString}
              level={hmi.preProcessStatus}
              time={timestampMsToTimeString(Date.now() / 1000)}
            >
            </MonitorItem>
          </ul>
          </div>
        </div>
    );
  }
}
