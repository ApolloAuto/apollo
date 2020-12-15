import {
  observable, action, computed, extendObservable,
} from 'mobx';
import _ from 'lodash';

import WS from 'store/websocket';
import UTTERANCE from 'store/utterance';
import RENDERER from 'renderer';

const TELEOP_MODE = Object.freeze({
  CAR: 'Car Teleop',
  CONSOLE: 'Console Teleop',
});

export default class HMI {
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

    @observable enableStartAuto = false;

    @observable dockerImage = 'unknown';

    @observable isCoDriver = false;

    @observable isMute = false;

    displayName = {};

    utmZoneId = 10;

    //@observable translationChanged=false;//默认没有changed

    @observable isCalibrationMode = false;

    @observable isSensorCalibrationMode = false;

    @observable dataCollectionUpdateStatus = observable.map();

    @observable dataCollectionProgress = observable.map();

    @observable lidars=observable.map();

    @observable cameras=observable.map();

    @observable mainSensor = 'none';

    @observable updateConfiguration = false;

    @observable canStartPreProcess=true;//随时可以start

    @observable endPreProcess=false;//不能end

    @observable preProcessStatus = 2;//正常通知图标

    @observable logString='';

    @observable preProcessProgress=0;

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
        this.isCalibrationMode = (newStatus.currentMode.toLowerCase().includes('calibration'));
        this.isSensorCalibrationMode = (newStatus.currentMode.toLowerCase().includes('sensor calibration'));
        if (this.currentMode !== newStatus.currentMode) {
          this.resetDataCollectionProgress();
          this.resetPreProcessProgress();
          this.currentMode = newStatus.currentMode;
          if (this.isSensorCalibrationMode) {
            this.updateConfiguration = true;
            //this.updateLidarConfiguration = this.inLidarIMUSensorCalibrationMode;
            //this.updateCameraLidarConfiguration = this.inCameraLidarSensorCalibrationMode;
            //this.translationChanged = false;
            //this.preConditionModule = 'Recorder';
            //这个Record这么写不优美 待细化
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
        if (this.isCalibrationMode && this.currentVehicle !== newStatus.currentVehicle) {
          this.resetDataCollectionProgress();
        }
        if (this.isSensorCalibrationMode && this.currentVehicle !== newStatus.currentVehicle) {
          //this.updateLidarConfiguration = true;
          //this.updateCameraLidarConfiguration = true;
          this.updateConfiguration = true;
          //this.translationChanged = false;
          this.resetPreProcessProgress();//有的地方可以归在标定的地方
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
        const newKeyList = JSON.stringify(Object.keys(newStatus.monitoredComponents).sort());
        const curKeyList = JSON.stringify(this.componentStatus.keys().sort());
        if (newKeyList !== curKeyList) {
          this.componentStatus.clear();
        }
        for (const key in newStatus.monitoredComponents) {
          this.componentStatus.set(key, newStatus.monitoredComponents[key]);
        }
        if (this.startMonitorRecorderProcess && ! this.allMonitoredComponentSuccess) {
          //正在检测且检测出非success 不报错误信息  直接关
          this.toggleModule(this.preConditionModule);
        }
      }

      if (typeof newStatus.passengerMsg === 'string') {
        UTTERANCE.speakRepeatedly(newStatus.passengerMsg);
      }
    }

    @action update(world) {
      this.enableStartAuto = world.engageAdvice === 'READY_TO_ENGAGE';
    }

    updateVehicleParam(vehicleParam) {
      this.vehicleParam = vehicleParam;
      RENDERER.adc.resizeCarScale(this.vehicleParam.length / this.defaultVehicleSize.length,
        this.vehicleParam.width / this.defaultVehicleSize.width,
        this.vehicleParam.height / this.defaultVehicleSize.height);
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

    @computed get shouldDisplayNavigationMap() {
      return this.inNavigationMode || this.inTeleopMode;
    }

    @computed get allMonitoredComponentSuccess() {
      //这个地方打印一下 componentstatus看一下是true Or false
      return this.isCalibrationMode && _.every(this.componentStatus, ['status', 'SUCCESS']);
    }

    @computed get preConditionModule() {
      return (this.isSensorCalibrationMode) ? 'Recorder' : 'none';
    }

    @computed get startMonitorRecorderProcess() {
      return this.isSensorCalibrationMode && this.moduleStatus.get('Recorder');
    }

    @action resetDataCollectionProgress() {
      this.dataCollectionUpdateStatus.clear();
      this.dataCollectionProgress.clear();
    }

    @action resetPreProcessProgress() {
      this.lidars.clear();
      this.cameras.clear();
      this.canStartPreProcess = true;
      this.preProcessStatus = 2;//正常就是通知的模式
      this.logString = '';
      this.endPreProcess = false;
      this.preProcessProgress = 0;
    }

    @action updateDataCollectionProgress(data) {
      Object.keys(data).sort().forEach((scenarioName) => {
        if (!this.dataCollectionProgress.has(scenarioName)) {
          this.dataCollectionProgress.set(scenarioName, observable.map());
          this.dataCollectionUpdateStatus.set(scenarioName, observable.map());
        }
        const categoryProgress = this.dataCollectionProgress.get(scenarioName);
        const categoryStatus = this.dataCollectionUpdateStatus.get(scenarioName);
        const scenario = data[scenarioName];
        Object.keys(scenario).sort().forEach((categoryName) => {
          const isUpdated = categoryProgress.get(categoryName) !== scenario[categoryName];
          categoryProgress.set(categoryName, scenario[categoryName]);
          categoryStatus.set(categoryName, isUpdated);
        });
      });
    }

  @action updatePreprocessProgress(data) {
      if (this.updateConfiguration) {
        if (data.lidarConfig) {
          data.lidarConfig.map(lidar => {
            this.lidars.set(lidar.sensorName, lidar.translation);
          });
        }
        if (data.cameraConfig) {
          data.cameraConfig.map(camera => {
            this.cameras.set(camera.cameraName, camera.translation);
          });
        }
        //数据Ok Main sensor名字唯一
        this.mainSensor = data.mainSensor;
        this.updateConfiguration = false;
      }
      if (data.progress) {
        //有进度更新进度 有logString 更新logString 状态进行捕获 如果是success 或者failed
        this.preProcessProgress = _.get(data, 'progress.percentage');
        this.logString = _.get(data, 'progress.logString');
        //这个地方去读状态只要发现是SUCCESS 或者FATAL 则endPreProcess置为true 该end了
        console.log(_.get(data, 'progress.status'));
        this.preProcessStatus = _.get(data, 'progress.status');
        this.endPreProcess = (this.preProcessStatus === 1) || (this.preProcessStatus === 0);
        //this.endPreProcess = ['SUCCESS', 'FAIL'].includes(_.get(data, 'progress.status'));
        if (!this.canStartPreProcess && this.endPreProcess) {
          //已经进入这个过程 需要监管是否可以end
          this.canStartPreProcess = true;
        }
      }
    }

    @action changeTranslation(name, index, val, isLidar) {
    isLidar ? _.set(this.lidars.get(name), index, val)
      : _.set(this.cameras.get(name), index, val);
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
