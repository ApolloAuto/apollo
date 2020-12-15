import React from 'react';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';
import { inject,observer } from 'mobx-react';

import WS from 'store/websocket';
import ScenarioCollectionMonitor from 'components/DataCollectionMonitor/ScenarioCollectionMonitor';
import GuideText from 'components/common/GuideText';
import SensorCalibrationConfiguration from './SensorCalibrationConfiguration';
import { MonitorItem } from '../Tasks/Console';
import { timestampMsToTimeString } from 'utils/misc';
import Selector from 'components/Header/Selector';

@inject('store') @observer
export default class FuelClient extends React.Component {
  constructor(props) {
    super(props);

    this.handlePreprocess = this.handlePreprocess.bind(this);
  }

  handlePreprocess() {
    // 出错 成功 初始化
    const hmi = this.props.store.hmi;
    if (hmi.canStartPreProcess) {
      hmi.canStartPreProcess = false;
      const data = {};
      //不考虑是否改变 每次都发
      if (hmi.inCameraLidarSensorCalibrationMode) {
        const select = document.getElementsByClassName('camera-selector')[0];
        const key = select.options[select.selectedIndex].value;
        _.set(data, 'camera_config.camera_name', key);
        _.set(data, 'camera_config.translation', this.props.store.hmi.cameras.get(key));
      }
      const lidar_configs = [];
      this.props.store.hmi.lidars.forEach((value, key) => {
        lidar_configs.push({
          sensor_name: key,
          translation: value,
        });
      });
      _.set(data, 'lidar_config', lidar_configs);
      _.set(data, 'task_type', hmi.currentMode);
      _.set(data, 'main_sensor', hmi.mainSensor);
      WS.startPreProcessData(data);
    }
  }

  render() {
    const { mode, preProcessProgress, cameras,lidars,mainSensor,
      toggleTranslationChange, inCameraLidarSensorCalibrationMode } = this.props;
    const hmi = this.props.store.hmi;

    return (
      <div className="monitor data-collection-monitor">
        <Tabs>
          <TabList>
            <Tab>Instruction</Tab>
            <Tab>Configuration</Tab>
          </TabList>
        <TabPanel>
        <GuideText mode={mode}></GuideText>
        </TabPanel>
          <TabPanel>
         {inCameraLidarSensorCalibrationMode && (!_.isEmpty(cameras)) && (
          <Selector
            className="camera-selector"
            name="choose calibration camera"
            options={cameras.keys()}
            currentOption={cameras.keys()[0]}
            onChange={(event) => {
              //WS.changeSetupMode(event.target.value);
              console.log(event);
            }}
          />
         )}
        <SensorCalibrationConfiguration
          lidars={hmi.lidars}
          mainSensor={hmi.mainSensor}
          cameras={hmi.cameras}
        >
        </SensorCalibrationConfiguration>
        <div className="preprocess-bar category">
          <div className="category-description">
            <button
              className="preprocess-btn"
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
              level="ERROR"//level等加了status字段改进一下
              time={timestampMsToTimeString(Date.now() / 1000)}
            >
            </MonitorItem>
          </ul>
          </div>
          </TabPanel>
          </Tabs>
      </div>
    );
  }
}