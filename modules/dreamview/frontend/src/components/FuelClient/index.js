import React from 'react';
import { Tab, Tabs, TabList, TabPanel } from 'react-tabs';
import { inject, observer } from 'mobx-react';
import classNames from 'classnames';
import _ from 'lodash';

import WS from 'store/websocket';
import GuideText from 'components/common/GuideText';
import SensorCalibrationConfiguration from './SensorCalibrationConfiguration';
import { MonitorItem } from '../Tasks/Console';
import { timestampMsToTimeString } from 'utils/misc';

@inject('store') @observer
export default class FuelClient extends React.Component {
  constructor(props) {
    super(props);

    this.handlePreprocess = this.handlePreprocess.bind(this);
  }

  handlePreprocess() {
    const hmi = this.props.store.hmi;
    hmi.isPreprocess = !hmi.isPreprocess;
    if (!hmi.preprocessIsRunning) {
      const data = {};
      if (hmi.inCameraLidarSensorCalibrationMode) {
        const internal_conf_input = _.map(
          document.getElementsByClassName('camera-internal-configuration-d'),
          'value',
        );
        const camera_internal_conf = internal_conf_input.map((x) =>
          parseFloat(x)
        );
        if (_.findIndex(camera_internal_conf, (x) => isNaN(x)) !== -1) {
          alert('Please input all camera internal configurations');
          return;
        }
        _.set(data, 'camera_config.D', _.slice(camera_internal_conf, 0, 5));
        _.set(data, 'camera_config.K', _.slice(camera_internal_conf, 5, 14));
        _.set(data, 'camera_config.translation', _.get(hmi.camera, 'translation'));
      }
      if (!_.isEmpty(hmi.lidars)) {
        const lidar_configs = [];
        hmi.lidars.forEach((value, key) => {
          lidar_configs.push({
            sensor_name: key,
            translation: value,
          });
        });
        _.set(data, 'lidar_config', lidar_configs);
      }
      _.set(data, 'main_sensor', hmi.mainSensor);
      WS.startPreprocessData(data, 'SensorCalibrationPreprocess');
      hmi.preprocessStarted = true;
      hmi.unexpectedAborted = false;
    }
  }

  render() {
    const hmi = this.props.store.hmi;

    return (
      <div className="monitor data-collection-monitor">
        <Tabs>
          <TabList>
            <Tab>Instruction</Tab>
            <Tab>Configuration</Tab>
          </TabList>
          <TabPanel>
            <GuideText mode={hmi.currentMode}></GuideText>
          </TabPanel>
          <TabPanel>
            <SensorCalibrationConfiguration
              lidars={hmi.lidars}
              mainSensor={hmi.mainSensor}
              camera={hmi.camera}
              componentStatus={hmi.componentStatus}
            ></SensorCalibrationConfiguration>
            <div className="preprocess-bar category">
              <div className="category-description">
                <button
                  className={classNames({
                    'preprocess-btn': true,
                    'button-active': hmi.isPreprocess,
                  })}
                  onClick={this.handlePreprocess}
                >
                  Preprocess
                </button>
              </div>
              <div className="category-progress-background">
                <span
                  className={
                    true ? 'category-completed' : 'category-in-progress'
                  }
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
                ></MonitorItem>
              </ul>
            </div>
          </TabPanel>
        </Tabs>
      </div>
    );
  }
}
