import STORE from 'store';
import Worker from 'utils/webworker.js';
import { action } from 'mobx';

export default class PluginWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.worker = new Worker();
  }

  checkWsConnection() {
    if (this.websocket.readyState === this.websocket.OPEN) {
      return this;
    }
    return this.initialize();
  }

  initialize() {
    try {
      // The connection is not initiated in the connected state
      if (this.websocket && this.websocket.readyState === this.websocket.OPEN) {
        return this;
      }
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return this;
    }
    this.websocket.addEventListener('message', (event) => {
      this.worker.postMessage({
        source: 'point_cloud',
        data: event.data,
      });
    });
    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };
    this.worker.addEventListener('message', (event) => {
      if (event.data.type === 'PluginResponse') {
        const message = event.data;
        const businessData =  message.data?.info || {};

        switch (message.data.name) {
          case 'StudioConnectorCertStatus':
            const status = businessData.code;
            if (status === 0) {
              this.getScenarioSetList();
              this.getDynamicsModelList();
              this.getRecordList();
            }
            STORE.studioConnector.checkCertificate(status);
            break;
          // Get Scenario Set Info Success
          case 'GetScenarioSetListSuccess':
            STORE.studioConnector.updateRemoteScenarioSetList(
              businessData.data ?? {},
            );
            break;
          // Get Scenario Set Info Failed
          case 'GetScenarioSetListFail':
            STORE.studioConnector.updateRemoteScenarioSetList(
              businessData.data ?? {},
            );
            break;
          case 'DownloadScenarioSet':
            STORE.studioConnector.updateRemoteScenarioSetStatus(
              businessData?.data?.resource_id,
              'downloaded',
              businessData?.data?.status,
            );
            // this.getScenarioSetList();
            break;
          case 'DownloadScenarioSetFail':
            STORE.studioConnector.updateRemoteScenarioSetStatus(
              businessData?.data?.resource_id,
              'fail',
              businessData?.data?.error_msg,
            );
            break;
          case 'GetDynamicModelListSuccess':
            STORE.studioConnector.updateRemoteDynamicsModelList(
              businessData.data ?? {}
            );
            break;
          case 'GetDynamicModelListFail':
            STORE.studioConnector.updateRemoteDynamicsModelList(
              businessData.data ?? {}
            );
            break;
          case 'DownloadDynamicModelSuccess':
            STORE.studioConnector.updateRemoteDynamicsModelStatus(
              businessData?.data?.dynamic_model_name,
              businessData?.data?.status,
            );
            // this.getDynamicsModelList();
            break;
          case 'DownloadDynamicModelFail':
            STORE.studioConnector.updateRemoteDynamicsModelStatus(
              businessData?.data?.dynamic_model_name,
              'fail',
              businessData?.data?.error_msg,
            );
            break;
          case 'GetRecordsList':
            STORE.studioConnector.updateRemoteRecordsList(
              businessData?.data ?? {}
            );
            break;
          case 'GetRecordsListSuccess':
            STORE.studioConnector.updateRemoteRecordsList(
              businessData?.data ?? {}
            );
            break;
          case 'GetRecordListFail':
            STORE.studioConnector.updateRemoteRecordsList(
              businessData?.data ?? {},
            );
            break;
          // 下载record成功
          case 'UpdateRecordToStatus':
            STORE.studioConnector.updateRemoteRecordStatus(
              businessData?.data?.resource_id,
              businessData?.data?.status,
            );
            break;
          case 'DownloadRecordFail':
            STORE.studioConnector.updateRemoteRecordStatus(
              businessData?.data?.record_id,
              'fail',
              businessData?.data?.error_msg,
            );
            break;
          case 'GetVehicleInfoSuccess':
            STORE.studioConnector.updateVehicleInfo(
              businessData.data ?? {},
              2,
            );
            break;
          case 'GetVehicleInfoFail':
            STORE.studioConnector.updateVehicleInfo(
              businessData.data ?? {},
              3,
            );
            break;
          case 'RefreshVehicleConfigSuccess':
            STORE.studioConnector.refreshVehicleConfig(
              businessData.data ?? {},
              2,
            );
        }
      }
    });
    return this;
  }

  checkCertificate() {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'CheckCertStatus',
          source: 'dreamview',
          info: '',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  getScenarioSetList() {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'GetScenarioSetList',
          source: 'dreamview',
          info: '',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  downloadScenarioSetById(scenarioSetId) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'DownloadScenarioSet',
          source: 'dreamview',
          info: scenarioSetId,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  // 下载record
  downloadRecord(id) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'DownloadRecord',
          source: 'dreamview',
          info: id,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  // 获取动力学模型列表
  getDynamicsModelList() {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'GetDynamicModelList',
          source: 'dreamview',
          info: '',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  // 下载动力学模型
  downloadDynamicsModel(modelName) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'DownloadDynamicModel',
          source: 'dreamview',
          info: modelName,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  // 获取数据包列表
  getRecordList() {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'GetRecordsList',
          source: 'dreamview',
          info: '',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  getVehicleInfo() {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'GetVehicleInfo',
          source: 'dreamview',
          info: '',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return this;
  }

  /**
   * refresh vehicle config
   */
  refreshVehicleConfig(vehicle_id) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'RefreshVehicleConfig',
          source: 'dreamview',
          info: vehicle_id,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'RefreshVehicleConfigSuccess':
              resolve();
              break;
            case 'RefreshVehicleConfigFail':
              reject();
              break;
          }
        }
      });
    });
  }

  /**
   * reset vehicle config
   */
  resetVehicleConfig(vehicle_id) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'ResetVehicleConfig',
          source: 'dreamview',
          info: vehicle_id,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'ResetVehicleConfigSuccess':
              resolve();
              break;
            case 'ResetVehicleConfigFail':
              reject();
              break;
          }
        }
      });
    });
  }

  /**
   * upload vehicle config
   */
  uploadVehicleConfig(vehicle_id) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'UploadVehicleConfig',
          source: 'dreamview',
          info: vehicle_id,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'UploadConfigSuccess':
              resolve();
              break;
            case 'UploadConfigFail':
              reject();
              break;
          }
        }
      });
    });
  }

  /** GetV2xInfo */
  getV2xInfo() {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'GetV2xInfo',
          source: 'dreamview',
          info: '',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'GetV2xInfoSuccess':
              resolve(JSON.parse(message.data.info ?? '{}'));
              break;
            case 'GetV2xInfoFail':
              reject(JSON.parse(message.data.info ?? '{}'));
              break;
          }
        }
      });
    });
  }

  /** RefreshV2xConf */
  refreshV2xConf(v2xId) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'RefreshV2xConf',
          source: 'dreamview',
          info: v2xId,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'RefreshV2xConfSuccess':
              resolve();
              break;
            case 'RefreshV2xConfFail':
              reject();
              break;
          }
        }
      });
    });
  }

  /** ResetV2xConf */
  resetV2xConf(v2xId) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'ResetV2xConf',
          source: 'dreamview',
          info: v2xId,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'ResetV2xConfSuccess':
              resolve();
              break;
            case 'ResetV2xConfFail':
              reject();
              break;
          }
        }
      });
    });
  }

  /** UploadV2xConf */
  uploadV2xConf(v2xId) {
    this.websocket.send(
      JSON.stringify({
        type: 'PluginRequest',
        data: {
          name: 'UploadV2xConf',
          source: 'dreamview',
          info: v2xId,
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        },
      }),
    );
    return new Promise((resolve, reject) => {
      this.worker.addEventListener('message', (event) => {
        if (event.data.type === 'PluginResponse') {
          const message = event.data;
          switch (message.data.name) {
            case 'UploadV2xSuccess':
              resolve();
              break;
            case 'UploadV2xFail':
              reject();
              break;
          }
        }
      });
    });
  }
}
