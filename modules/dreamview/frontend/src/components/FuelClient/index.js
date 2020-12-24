import React from 'react';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';
import { inject,observer } from 'mobx-react';

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
    // 出错 成功 初始化
    const hmi = this.props.store.hmi;
    if (hmi.canStartPreprocess) {
      hmi.canStartPreprocess = false;
      const data = {};
      //不考虑是否改变 每次都发
      //这个地方原来要传选中camera的config（name和translation）
      //
      if (hmi.inCameraLidarSensorCalibrationMode) {
        const internal_conf_input = _.map(document.getElementsByClassName('camera-internal-configuration-d'),'value');
        const camera_internal_conf = internal_conf_input.map(x => parseFloat(x));
        if (_.findIndex(camera_internal_conf, (x) => isNaN(x)) !== -1) {
          alert('Please input all camera internal configurations');
          hmi.canStartPreprocess = true;
          return;
        }
        _.set(data, 'camera_config.D', _.slice(camera_internal_conf, 0, 5));
        _.set(data, 'camera_config.K', _.slice(camera_internal_conf, 5, 14));
        _.set(data, 'camera_config.R', _.slice(camera_internal_conf, 14, 26));
        _.set(data, 'camera_config.translation', hmi.camera.translation);
      }
      if (!_.isEmpty(this.props.store.hmi.lidars)) {
        const lidar_configs = [];
        this.props.store.hmi.lidars.forEach((value, key) => {
          lidar_configs.push({
            sensor_name: key,
            translation: value,
          });
        });
        _.set(data, 'lidar_config', lidar_configs);
      }
      _.set(data, 'main_sensor', hmi.mainSensor);
      WS.startPreprocessData(data, 'SensorCalibrationPreprocess');
    }
  }

  renderCameraInternalInput(number) {
    const result = [];
    for (let i = 0; i < number; i++) {
      result.push(<input type="number" className="camera-internal-configuration-d" key={`camera${i}`}></input>);
    }
    return result;
  }

  render() {
    const { mode, inCameraLidarSensorCalibrationMode } = this.props;
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
        <SensorCalibrationConfiguration
          lidars={hmi.lidars}
          mainSensor={hmi.mainSensor}
          camera={hmi.camera}
        >
            </SensorCalibrationConfiguration>
            {inCameraLidarSensorCalibrationMode &&
               (<div>
              Camera-Lidar内参设定
              <div>D:
              {this.renderCameraInternalInput(5)}
                 </div>
              <div> K:
              {this.renderCameraInternalInput(9)}
               </div>
              <div>R:
              {this.renderCameraInternalInput(9)}
                 </div>
             </div>)
        }
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
              style={{ width: `${hmi.preprocessProgress}%` }}
            />
          </div>
        </div>
        <div className="preprocess-msg">
          <ul className="preprocess-console">
            <MonitorItem
              text={hmi.logString}
              level={hmi.preprocessStatus}
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
