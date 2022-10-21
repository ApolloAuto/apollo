import STORE from 'store';
import RENDERER from 'renderer';
import MAP_NAVIGATOR from 'components/Navigation/MapNavigator';
import UTTERANCE from 'store/utterance';
import Worker from 'utils/webworker.js';

export default class RealtimeWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.simWorldUpdatePeriodMs = 100;
    this.simWorldLastUpdateTimestamp = 0;
    this.mapUpdatePeriodMs = 1000;
    this.mapLastUpdateTimestamp = 0;
    this.updatePOI = true;
    this.updateDefaultRoutingPoints = true;
    this.routingTime = undefined;
    this.currentMode = null;
    this.worker = new Worker();
    this.pointcloudWS = null;
    this.requestHmiStatus = this.requestHmiStatus.bind(this);
    this.updateParkingRoutingDistance = true;
  }

  initialize() {
    try {
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }
    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'realtime',
        data: event.data,
      });
    };
    this.worker.onmessage = (event) => {
      const message = event.data;
      switch (message.type) {
        case 'HMIStatus':
          STORE.hmi.updateStatus(message.data);
          STORE.studioConnector.updateLocalScenarioInfo(message.data);
          RENDERER.updateGroundImage(STORE.hmi.currentMap);
          break;
        case 'VehicleParam':
          STORE.hmi.updateVehicleParam(message.data);
          break;
        case 'SimControlStatus':
          STORE.setOptionStatus('enableSimControl', message.enabled);
          break;
        case 'SimWorldUpdate':
          this.checkMessage(message);

          const isNewMode = (this.currentMode
            && this.currentMode !== STORE.hmi.currentMode);
          const isNavigationModeInvolved = (this.currentMode === 'Navigation'
            || STORE.hmi.currentMode === 'Navigation');
          this.currentMode = STORE.hmi.currentMode;
          if (STORE.hmi.shouldDisplayNavigationMap) {
            if (MAP_NAVIGATOR.isInitialized()) {
              MAP_NAVIGATOR.update(message);
            }

            if (STORE.hmi.inNavigationMode) {
              // In navigation mode, the coordinate system is FLU and
              // relative position of the ego-car is (0, 0). But,
              // absolute position of the ego-car is needed in MAP_NAVIGATOR.
              message.autoDrivingCar.positionX = 0;
              message.autoDrivingCar.positionY = 0;
              message.autoDrivingCar.heading = 0;

              RENDERER.coordinates.setSystem('FLU');
              this.mapUpdatePeriodMs = 100;
            }
          } else {
            RENDERER.coordinates.setSystem('ENU');
            this.mapUpdatePeriodMs = 1000;
          }

          STORE.update(message, isNewMode);
          RENDERER.maybeInitializeOffest(
            message.autoDrivingCar.positionX,
            message.autoDrivingCar.positionY,
            // Updating offset only if navigation mode is involved since
            // its coordination system is different from rest of the modes.
            isNewMode && isNavigationModeInvolved,
          );
          RENDERER.updateWorld(message);
          this.updateMapIndex(message);
          if (this.routingTime !== message.routingTime) {
            // A new routing needs to be fetched from backend.
            this.requestRoutePath();
            this.routingTime = message.routingTime;
          }
          break;
        case 'MapElementIds':
          RENDERER.updateMapIndex(message.mapHash,
            message.mapElementIds, message.mapRadius);
          break;
        case 'DefaultEndPoint':
          STORE.routeEditingManager.updateDefaultRoutingEndPoint(message);
          break;
        case 'DefaultRoutings':
          STORE.routeEditingManager.updateDefaultRoutingPoints(message);
          break;
        case 'AddDefaultRoutingPath':
          // used for user-defined routing: default routing,park and go routing
          STORE.routeEditingManager.addDefaultRoutingPath(message);
          break;
        case 'RoutePath':
          RENDERER.updateRouting(message.routingTime, message.routePath);
          break;
        case 'RoutingPointCheckResult':
          if (message.error) {
            RENDERER.removeInvalidRoutingPoint(message.pointId, message.error);
          }
          break;
        case 'DataCollectionProgress':
          if (message) {
            STORE.hmi.updateDataCollectionProgress(message.data);
          }
          break;
        case 'PreprocessProgress':
          if (message) {
            STORE.hmi.updatePreprocessProgress(message.data);
          }
          break;
        case 'ParkingRoutingDistance':
          if (message) {
            STORE.routeEditingManager.updateParkingRoutingDistance(message.threshold);
          }
          break;
      }
    };
    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed, close_code: ${event.code}`);

      // If connection has been lost for more than 10 sec, send the error message every 2 sec
      const now = new Date().getTime();
      const lossDuration = now - this.simWorldLastUpdateTimestamp;
      const alertDuration = now - STORE.monitor.lastUpdateTimestamp;
      if (this.simWorldLastUpdateTimestamp !== 0
        && lossDuration > 10000 && alertDuration > 2000) {
        const message = 'Connection to the server has been lost.';
        STORE.monitor.insert('FATAL', message, now);
        if (UTTERANCE.getCurrentText() !== message || !UTTERANCE.isSpeaking()) {
          UTTERANCE.speakOnce(message);
        }
      }

      this.initialize();
    };

    // Request simulation world every 100ms.
    clearInterval(this.timer);
    this.timer = setInterval(() => {
      if (this.websocket.readyState === this.websocket.OPEN) {
        // Load default routing end point.
        if (this.updatePOI) {
          this.requestDefaultRoutingEndPoint();
          this.updatePOI = false;
        }
        // Load default routing points user defined
        if (this.updateDefaultRoutingPoints) {
          this.requestDefaultRoutingPoints();
          this.updateDefaultRoutingPoints = false;
        }

        if (this.pointcloudWS.isEnabled()) {
          this.pointcloudWS.requestPointCloud();
        }
        this.requestSimulationWorld(STORE.options.showPNCMonitor);
        if (this.updateParkingRoutingDistance) {
          this.requestParkingRoutingDistance();
          this.updateParkingRoutingDistance = false;
        }
        if (STORE.hmi.isVehicleCalibrationMode) {
          this.requestDataCollectionProgress();
          this.requestPreprocessProgress();
        }
        if (STORE.hmi.isSensorCalibrationMode) {
          this.requestPreprocessProgress();
        }
      }
    }, this.simWorldUpdatePeriodMs);
  }

  checkWsConnection() {
    if (this.websocket.readyState === this.websocket.OPEN) {
      return this;
    }
    return this.initialize();
  }

  updateMapIndex(message) {
    const now = new Date();
    const duration = now - this.mapLastUpdateTimestamp;
    if (message.mapHash && duration >= this.mapUpdatePeriodMs) {
      RENDERER.updateMapIndex(message.mapHash, message.mapElementIds, message.mapRadius);
      this.mapLastUpdateTimestamp = now;
    }
  }

  checkMessage(world) {
    const now = new Date().getTime();
    const duration = now - this.simWorldLastUpdateTimestamp;
    if (this.simWorldLastUpdateTimestamp !== 0 && duration > 200) {
      console.warn(`Last sim_world_update took ${duration}ms`);
    }
    if (this.secondLastSeqNum === world.sequenceNum) {
      // Receiving multiple duplicated simulation_world messages
      // indicates a backend lag.
      console.warn('Received duplicate simulation_world:', this.lastSeqNum);
    }
    this.secondLastSeqNum = this.lastSeqNum;
    this.lastSeqNum = world.sequenceNum;
    this.simWorldLastUpdateTimestamp = now;
  }

  requestSimulationWorld(requestPlanningData) {
    this.websocket.send(JSON.stringify({
      type: 'RequestSimulationWorld',
      planning: requestPlanningData,
    }));
  }

  checkRoutingPoint(point) {
    const request = {
      type: 'CheckRoutingPoint',
      point,
    };
    this.websocket.send(JSON.stringify(request));
  }

  requestMapElementIdsByRadius(radius) {
    this.websocket.send(JSON.stringify({
      type: 'RetrieveMapElementIdsByRadius',
      radius,
    }));
  }

  requestRoute(start, start_heading, waypoint, end, parkingInfo) {
    const request = {
      type: 'SendRoutingRequest',
      start,
      end,
      waypoint,
    };

    if (parkingInfo) {
      request.parkingInfo = parkingInfo;
    }

    if (start_heading) {
      request.start.heading = start_heading;
    }
    this.websocket.send(JSON.stringify(request));
  }

  requestDefaultCycleRouting(start, start_heading, waypoint, end, cycleNumber) {
    const request = {
      type: 'SendDefaultCycleRoutingRequest',
      start,
      end,
      waypoint,
      cycleNumber,
    };
    if (start_heading) {
      request.start.heading = start_heading;
    }
    this.websocket.send(JSON.stringify(request));
  }

  requestDefaultRoutingEndPoint() {
    this.websocket.send(JSON.stringify({
      type: 'GetDefaultEndPoint',
    }));
  }

  requestDefaultRoutingPoints() {
    this.websocket.send(JSON.stringify({
      type: 'GetDefaultRoutings',
    }));
  }

  requestParkingRoutingDistance() {
    this.websocket.send(JSON.stringify({
      type: 'GetParkingRoutingDistance',
    }));
  }

  resetBackend() {
    this.websocket.send(JSON.stringify({
      type: 'Reset',
    }));
  }

  dumpMessages() {
    this.websocket.send(JSON.stringify({
      type: 'Dump',
    }));
  }

  changeSetupMode(mode) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_MODE',
      value: mode,
    }));
  }

  changeMap(map) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_MAP',
      value: map,
    }));
    this.updatePOI = true;
    this.updateDefaultRoutingPoints = true;
  }

  changeVehicle(vehicle) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_VEHICLE',
      value: vehicle,
    }));
  }

  loadLoocalScenarioSets() {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'LOAD_SCENARIOS',
    }));
  }

  /**
   *
   * @param scenarioId string
   */
  changeScenario(scenarioId) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_SCENARIO',
      value: scenarioId,
    }));
  }

  /**
   *
   * @param scenarioSetId string
   */
  changeScenarioSet(scenarioSetId) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_SCENARIO_SET',
      value: scenarioSetId,
    }));

    // 切换场景集后，需要重新置空当前场景
    this.changeScenario('');
  }

  deleteScenarioSet(scenarioSetId) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'DELETE_SCENARIO_SET',
      value: scenarioSetId,
    }));
  }

  getDymaticModelList() {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'LOAD_DYNAMIC_MODELS',
    }));
  }

  changeDynamicModel(model) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_DYNAMIC_MODEL',
      value: model,
    }));
  }

  switchToDefaultDynamicModel() {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'CHANGE_DYNAMIC_MODEL',
      value: 'Simulation Perfect Control',
    }));
  }

  deleteDynamicModels(dynamicModelId) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: 'DELETE_DYNAMIC_MODEL',
      value: dynamicModelId,
    }));
  }

  // 加载本地records
  loadLocalRecords() {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action:'LOAD_RECORDS',
    }));
  }

  // 选择本地records
  changeRecord(recordId) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action:'CHANGE_RECORD',
      value: recordId,
    }));
  }

  // 删除本地record
  deleteRecord(recordId) {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action:'DELETE_RECORD',
      value: recordId,
    }));
  }

  // 停止本地record播放
  stopRecord() {
    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action:'STOP_RECORD',
    }));
  }
  executeModeCommand(action) {
    if (!['SETUP_MODE', 'RESET_MODE', 'ENTER_AUTO_MODE'].includes(action)) {
      console.error('Unknown mode command found:', action);
      return;
    }

    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action,
    }));

    setTimeout(this.requestHmiStatus, 5000);
  }

  executeModuleCommand(moduleName, command) {
    if (!['START_MODULE', 'STOP_MODULE'].includes(command)) {
      console.error('Unknown module command found:', command);
      return;
    }

    this.websocket.send(JSON.stringify({
      type: 'HMIAction',
      action: command,
      value: moduleName,
    }));

    setTimeout(this.requestHmiStatus, 5000);
  }

  submitDriveEvent(eventTimeMs, eventMessage, eventTypes, isReportable) {
    this.websocket.send(JSON.stringify({
      type: 'SubmitDriveEvent',
      event_time_ms: eventTimeMs,
      event_msg: eventMessage,
      event_type: eventTypes,
      is_reportable: isReportable,
    }));
  }

  submitAudioEvent(eventTimeMs, obstacleId, audioType, movingResult, direction, isSirenOn) {
    this.websocket.send(JSON.stringify({
      type: 'SubmitAudioEvent',
      event_time_ms: eventTimeMs,
      obstacle_id: obstacleId,
      audio_type: audioType,
      moving_result: movingResult,
      audio_direction: direction,
      is_siren_on: isSirenOn,
    }));
  }

  toggleSimControl(enable) {
    this.websocket.send(JSON.stringify({
      type: 'ToggleSimControl',
      enable,
    }));
  }

  requestRoutePath() {
    this.websocket.send(JSON.stringify({
      type: 'RequestRoutePath',
    }));
  }

  requestHmiStatus() {
    this.websocket.send(JSON.stringify({
      type: 'HMIStatus',
    }));
  }

  publishNavigationInfo(data) {
    this.websocket.send(data);
  }

  requestDataCollectionProgress() {
    this.websocket.send(JSON.stringify({
      type: 'RequestDataCollectionProgress',
    }));
  }

  setPointCloudWS(pointcloudws) {
    this.pointcloudWS = pointcloudws;
  }

  saveDefaultRouting(routingName, points) {
    const request = {
      type: 'SaveDefaultRouting',
      name: routingName,
      point: points,
      routingType: 'defaultRouting',
    };
    this.websocket.send(JSON.stringify(request));
  }

  requestPreprocessProgress() {
    this.websocket.send(JSON.stringify({
      type: 'RequestPreprocessProgress',
    }));
  }

  startPreprocessData(data, type) {
    const request = {
      type,
    };
    if (data) {
      request.data = data;
    }
    this.websocket.send(JSON.stringify(request));
  }

  sendParkingRequest(
    start, start_heading, waypoint, end, parkingInfo) {
    const request = {
      type: 'SendParkingRoutingRequest',
      start,
      end,
      waypoint,
      parkingInfo,
    };
    if (start_heading) {
      request.start.heading = start_heading;
    }
    this.websocket.send(JSON.stringify(request));
  }
}
