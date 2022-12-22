import { observable, action, computed } from 'mobx';
import _ from 'lodash';

import WS from 'store/websocket';
import UTTERANCE from 'store/utterance';
import RENDERER from 'renderer';

const TELEOP_MODE = Object.freeze({
  CAR: 'Car Teleop',
  CONSOLE: 'Console Teleop',
});

export default class HMI {

  constructor(studioConnector) {
    this.studioConnector = studioConnector;
  }

  modes = [];

  @observable currentMode = 'none';

  vehicles = [];

  @observable currentVehicle = 'none';

  defaultVehicleSize = {
    height: 1.48,
    width: 2.11,
    length: 4.933,
  };

  vehicleParam = {
    frontEdgeToCenter: 3.89,
    backEdgeToCenter: 1.043,
    leftEdgeToCenter: 1.055,
    rightEdgeToCenter: 1.055,
    height: 1.48,
    width: 2.11,
    length: 4.933,
    steerRatio: 16,
    wheelBase: 2.8448,
  };

  maps = [];

  @observable currentMap = 'none';

  @observable moduleStatus = observable.map();

  @observable componentStatus = observable.map();

  @observable otherComponentStatus = observable.map();

  @observable enableStartAuto = false;

  @observable dockerImage = 'unknown';

  @observable isCoDriver = false;

  @observable isMute = false;

  @observable isPreprocess = false;

  displayName = {};

  utmZoneId = 10;

  @observable isVehicleCalibrationMode = false;

  @observable isSensorCalibrationMode = false;

  @observable dataCollectionUpdateStatus = observable.map();

  @observable dataCollectionProgress = observable.map();

  @observable lidars = observable.map();

  @observable camera = {};

  @observable mainSensor = 'none';

  @observable updateConfiguration = false;

  @observable preprocessStarted = false;

  @observable preprocessFinished = false;

  @observable unexpectedAborted = false;

  @observable preprocessStatus = 'UNKNOWN'; // Under normal situation

  @observable logString = '';

  @observable preprocessProgress = 0;

  @observable preprocessMonitorItemTimeStamp = null;

  @observable counter = 0;

  @observable dynamicModels = [];

  @observable currentDynamicModel = '';

  @observable records = {};

  @observable currentRecordId = '';

  /**
   *  // 0: 未设置
   *  // 1: DKIT_LITE
   *  // 2: DKIT_STANDARD
   *  // 3: DKIT_ADVANCED_NE_S
   *  // 4: DKIT_ADVANCED_SNE_R
   *  // 5: DKIT_LITE_S
   *  // 6: DKIT_STANDARD_S
   *  // 7: DKIT_CHALLENGE
   * @type {number}
   */
  @observable currentVehicleType = 0;

  // current camera channel
  @observable currentCameraSensorChannel = '';

  // current point cloud channel
  @observable currentPointCloudChannel = '';

  @action toggleCoDriverFlag() {
    this.isCoDriver = !this.isCoDriver;
  }

  @action toggleMuteFlag() {
    this.isMute = !this.isMute;
    UTTERANCE.setMute(this.isMute);
  }

  @action updateStatus(newStatus) {
    if (newStatus.dockerImage) {
      this.dockerImage = newStatus.dockerImage;
    }
    if (newStatus.utmZoneId) {
      this.utmZoneId = newStatus.utmZoneId;
    }

    if (newStatus.modes) {
      this.modes = newStatus.modes.sort();
    }
    if (newStatus.currentMode) {
      this.isVehicleCalibrationMode = newStatus.currentMode
        .toLowerCase()
        .includes('vehicle calibration');
      this.isSensorCalibrationMode = newStatus.currentMode
        .toLowerCase()
        .includes('sensor calibration');
      if (this.currentMode !== newStatus.currentMode) {
        this.resetDataCollectionProgress();
        this.resetSensorCalibrationConfiguration();
        this.resetPreprocessProgress();
        this.currentMode = newStatus.currentMode;
        if (this.isSensorCalibrationMode) {
          this.updateConfiguration = true;
        }
      }
    }

    if (newStatus.maps) {
      this.maps = newStatus.maps.sort();
    }
    if (newStatus.currentMap) {
      this.currentMap = newStatus.currentMap;
    }

    if (newStatus.vehicles) {
      this.vehicles = newStatus.vehicles.sort();
    }
    if (newStatus.currentVehicle) {
      if (
        this.isVehicleCalibrationMode &&
        this.currentVehicle !== newStatus.currentVehicle
      ) {
        this.resetDataCollectionProgress();
        this.resetPreprocessProgress();
      }
      if (
        this.isSensorCalibrationMode &&
        this.currentVehicle !== newStatus.currentVehicle
      ) {
        this.updateConfiguration = true;
        this.resetSensorCalibrationConfiguration();
        this.resetPreprocessProgress();
      }
      this.currentVehicle = newStatus.currentVehicle;
    }

    if (newStatus.modules) {
      const newKeyList = JSON.stringify(Object.keys(newStatus.modules).sort());
      const curKeyList = JSON.stringify(this.moduleStatus.keys().sort());
      if (newKeyList !== curKeyList) {
        this.moduleStatus.clear();
      }
      for (const key in newStatus.modules) {
        this.moduleStatus.set(key, newStatus.modules[key]);
      }
    }

    if (newStatus.monitoredComponents) {
      const newKeyList = JSON.stringify(
        Object.keys(newStatus.monitoredComponents).sort(),
      );
      const curKeyList = JSON.stringify(this.componentStatus.keys().sort());
      if (newKeyList !== curKeyList) {
        this.componentStatus.clear();
      }
      for (const key in newStatus.monitoredComponents) {
        this.componentStatus.set(key, newStatus.monitoredComponents[key]);
      }
      if (
        this.startMonitorRecorderProcess &&
        !this.allMonitoredComponentSuccess
      ) {
        this.toggleModule(this.preConditionModule);
      }
    }

    if (newStatus.otherComponents) {
      const newKeyList = JSON.stringify(
        Object.keys(newStatus.otherComponents).sort(),
      );
      const curKeyList = JSON.stringify(
        this.otherComponentStatus.keys().sort(),
      );
      if (newKeyList !== curKeyList) {
        this.otherComponentStatus.clear();
      }
      for (const key in newStatus.otherComponents) {
        this.otherComponentStatus.set(key, newStatus.otherComponents[key]);
      }
    }

    if (this.preprocessStarted && !this.preprocessIsRunning) {
      this.counter += 1; // use counter to delay time
      if (this.counter > 1) {
        if (!this.preprocessFinished) {
          this.unexpectedAborted = true;
        }
        this.counter = 0;
        this.preprocessStarted = false;
      }
    }

    if (typeof newStatus.passengerMsg === 'string') {
      UTTERANCE.speakRepeatedly(newStatus.passengerMsg);
    }

    if (newStatus.dynamicModels) {
      this.dynamicModels = newStatus.dynamicModels;
    }

    if (newStatus.currentDynamicModel) {
      this.currentDynamicModel = newStatus.currentDynamicModel;
    }

    this.records = newStatus.records;
    this.currentRecordId = newStatus.currentRecordId;

    if (newStatus.currentVehicleType) {
      this.currentVehicleType = newStatus.currentVehicleType;
    }

    if (newStatus.currentCameraSensorChannel) {
      this.currentCameraSensorChannel = newStatus.currentCameraSensorChannel;
    }

    if (newStatus.currentPointCloudChannel) {
      this.currentPointCloudChannel = newStatus.currentPointCloudChannel;
    }
  }

  @action update(world) {
    this.enableStartAuto = world.engageAdvice === 'READY_TO_ENGAGE';
  }

  updateVehicleParam(vehicleParam) {
    this.vehicleParam = vehicleParam;
    RENDERER.adc.resizeCarScale(
      this.vehicleParam.length / this.defaultVehicleSize.length,
      this.vehicleParam.height / this.defaultVehicleSize.height,
      this.vehicleParam.width / this.defaultVehicleSize.width,
    );
  }

  @action toggleModule(id) {
    this.moduleStatus.set(id, !this.moduleStatus.get(id));
    const command = this.moduleStatus.get(id) ? 'START_MODULE' : 'STOP_MODULE';
    WS.executeModuleCommand(id, command);
  }

  @computed get inNavigationMode() {
    return this.currentMode === 'Navigation';
  }

  @computed get inCarTeleopMode() {
    return this.currentMode === TELEOP_MODE.CAR;
  }

  @computed get inConsoleTeleopMode() {
    return this.currentMode === TELEOP_MODE.CONSOLE;
  }

  @computed get inTeleopMode() {
    return Object.values(TELEOP_MODE).includes(this.currentMode);
  }

  @computed get inCameraLidarSensorCalibrationMode() {
    return this.currentMode === 'Camera-Lidar Sensor Calibration';
  }

  @computed get isCalibrationMode() {
    return this.isSensorCalibrationMode || this.isVehicleCalibrationMode;
  }

  @computed get shouldDisplayNavigationMap() {
    return this.inNavigationMode || this.inTeleopMode;
  }

  @computed get allMonitoredComponentSuccess() {
    return (
      this.isCalibrationMode &&
      _.every(this.componentStatus.keys(), (key) => {
        return key === 'Recorder' || _.get(this.componentStatus.get(key), 'status') === 'OK';
      })
    );
  }

  @computed get preConditionModule() {
    return this.isCalibrationMode ? 'Recorder' : 'none';
  }

  @computed get startMonitorRecorderProcess() {
    return this.isSensorCalibrationMode && this.moduleStatus.get('Recorder');
  }

  @computed get canStartDataCollectionPreprocess() {
    return (
      this.isVehicleCalibrationMode &&
      _.some(
        this.dataCollectionProgress.get('Go Straight').values(),
        (x) => x > 0,
      )
    );
  }

  @computed get preprocessIsRunning() {
    return (
      this.isCalibrationMode &&
      this.otherComponentStatus &&
      _.get(this.otherComponentStatus.get('Preprocess'), 'status') === 'OK'
    );
  }

  @computed get startUpdateDataCollectionProgress() {
    return this.isVehicleCalibrationMode && this.moduleStatus.get('Recorder');
  }

  @action resetDataCollectionProgress() {
    this.dataCollectionUpdateStatus.clear();
    this.dataCollectionProgress.clear();
  }

  @action resetSensorCalibrationConfiguration() {
    this.lidars.clear();
    this.camera = {};
  }

  @action resetPreprocessProgress() {
    this.preprocessStarted = false;
    this.preprocessFinished = false;
    this.unexpectedAborted = false;
    this.preprocessStatus = 'UNKNOWN';
    this.logString = '';
    this.preprocessProgress = 0;
    this.preprocessMonitorItemTimeStamp = null;
  }

  @action updateDataCollectionProgress(data) {
    Object.keys(data)
      .sort()
      .forEach((scenarioName) => {
        if (!this.dataCollectionProgress.has(scenarioName)) {
          this.dataCollectionProgress.set(scenarioName, observable.map());
          this.dataCollectionUpdateStatus.set(scenarioName, observable.map());
        }
        const categoryProgress = this.dataCollectionProgress.get(scenarioName);
        const categoryStatus = this.dataCollectionUpdateStatus.get(
          scenarioName,
        );
        const scenario = data[scenarioName];
        Object.keys(scenario)
          .sort()
          .forEach((categoryName) => {
            const isUpdated =
              categoryProgress.get(categoryName) !== scenario[categoryName];
            categoryProgress.set(categoryName, scenario[categoryName]);
            categoryStatus.set(categoryName, isUpdated);
          });
      });
  }

  @action updatePreprocessProgress(data) {
    if (this.updateConfiguration) {
      if (data.lidarConfig) {
        data.lidarConfig.map((lidar) => {
          this.lidars.set(lidar.sensorName, lidar.translation);
        });
      }
      if (data.cameraConfig) {
        this.camera = data.cameraConfig;
      }
      this.mainSensor = data.mainSensor;
      this.updateConfiguration = false;
    }
    if (data.progress) {
      this.preprocessMonitorItemTimeStamp = new Date().getTime();
      if (this.unexpectedAborted) {
        this.preprocessStatus = 'FAIL';
        this.logString =
          'The preprocessing process has been aborted unexpectedly, please check nohup.out for reasons or try again.';
      } else {
        this.preprocessProgress = _.get(data, 'progress.percentage');
        this.logString = _.get(data, 'progress.logString');
        this.preprocessStatus = _.get(data, 'progress.status');
        if (['SUCCESS', 'FAIL'].includes(this.preprocessStatus)) {
          this.preprocessFinished = true;
        } else {
          this.preprocessFinished = false;
        }
      }
    }
  }

  // name:{x y z}
  @action changeTranslation(name, index, val, isLidar) {
    isLidar
      ? _.set(this.lidars.get(name), index, val)
      : _.set(this.camera, `translation.${index}`, val);
  }

  @action changeIntrinsic(name, index, val) {
    _.set(this.camera, `${name}[${index}]`, val);
  }

  rotate2DPoint({ x, y }, rotationInRad) {
    return {
      x: x * Math.cos(rotationInRad) - y * Math.sin(rotationInRad),
      y: x * Math.sin(rotationInRad) + y * Math.cos(rotationInRad),
    };
  }

  calculateCarPolygonPoints(positionX, positionY, headingInRad) {
    const config = this.vehicleParam;
    const polygonPoints = [
      { y: -config.leftEdgeToCenter, x: config.frontEdgeToCenter },
      { y: config.rightEdgeToCenter, x: config.frontEdgeToCenter },
      { y: config.rightEdgeToCenter, x: -config.backEdgeToCenter },
      { y: -config.leftEdgeToCenter, x: -config.backEdgeToCenter },
      { y: -config.leftEdgeToCenter, x: config.frontEdgeToCenter },
    ];

    polygonPoints.forEach((point) => {
      const newPoint = this.rotate2DPoint(point, headingInRad);
      point.x = positionX + newPoint.x;
      point.y = positionY + newPoint.y;
    });

    return polygonPoints;
  }
}
