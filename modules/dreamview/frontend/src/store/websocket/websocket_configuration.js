import STORE from 'store';
import Worker from 'utils/webworker.js';

export default class ConfigurationWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.worker = new Worker();
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
        source: 'account',
        data: event.data,
      });
    };

    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };

    this.worker.onmessage = (event) => {
      const message = event.data;
      switch (message.type) {
        case 'ConfigurationProfileStatus':
          STORE.configStatus.update(message.data);
          break;
        case 'DefaultVehicleProfileTarballDownloadSuccess':
          STORE.configStatus.forceAccountInfoReload();
          break;
        case 'DefaultVehicleProfileTarballDownloadFailed':
          STORE.configStatus.forceAccountInfoReload();
          break;
        case 'DefaultVehicleProfileTarballDownloadCanceled':
          STORE.configStatus.forceAccountInfoReload();
          break;
        case 'RequestVehicleProfileTarballUploadSuccess':
          STORE.configStatus.forceAccountInfoReload();
          break;
        case 'RequestVehicleProfileTarballUploadFailed':
          STORE.configStatus.forceAccountInfoReload();
          break;
        case 'RequestVehicleProfileTarballUploadCanceled':
          STORE.configStatus.forceAccountInfoReload();
          break;

        default:
          console.warn('Account WebSocket received unknown message:', message);
          break;
      }
    };
  }

  requestConfigDownload(profileid, vtype) {
    this.websocket.send(JSON.stringify({
      type: 'RequestConfigurationDownload',
      id: profileid,
      vtype: vtype,
    }));
  }

  cancelConfigDownload(profileid, vtype) {
    this.websocket.send(JSON.stringify({
      type: 'CancelConfigurationDownload',
      id: profileid,
      vtype: vtype,
    }));
  }

  refreshConfigStatus() {
    this.websocket.send(JSON.stringify({
      type: 'ConfigurationProfileStatus'
    }));
  }

  updateVehicleSn(vehicleSn) {
    this.websocket.send(JSON.stringify({
      type: 'UpdateVehicleSn',
      vin: vehicleSn,
    }));
  }

  requestVehicleProfileTarballDownload(vehicleId, profileId) {
    const params = {
      vehicleId,
    };
    if (!_.isNil(profileId) && !_.isEmpty(profileId)) {
      params.profileId = profileId;
    }
    this.websocket.send(JSON.stringify(Object.assign({
      type: 'RequestVehicleProfileTarballDownload',
    }, params)));
  }

  cancelVehicleProfileTarballDownload(vehicleId, profileId) {
    const params = {
      vehicleId,
    };
    if (!_.isNil(profileId) && !_.isEmpty(profileId)) {
      params.profileId = profileId;
    }
    this.websocket.send(JSON.stringify(Object.assign({
      type: 'CancelVehicleProfileTarballDownload',
    }, params)));
  }

  requestVehicleProfileTarballUpload(vehicleId, profileId) {
    const params = {
      vehicleId,
    };
    if (!_.isNil(profileId) && !_.isEmpty(profileId)) {
      params.profileId = profileId;
    }
    this.websocket.send(JSON.stringify(Object.assign({
      type: 'RequestVehicleProfileTarballUpload',
    }, params)));
  }

  cancelVehicleProfileTarballUpload(vehicleId, profileId) {
    const params = {
      vehicleId,
    };
    if (!_.isNil(profileId) && !_.isEmpty(profileId)) {
      params.profileId = profileId;
    }
    this.websocket.send(JSON.stringify(Object.assign({
      type: 'CancelVehicleProfileTarballUpload',
    }, params)));
  }

  requestVehicleProfileTarballReset(vehicleId, profileId) {
    const params = {
      vehicleId,
    };
    if (!_.isNil(profileId) && !_.isEmpty(profileId)) {
      params.profileId = profileId;
    }
    this.websocket.send(JSON.stringify(Object.assign({
      type: 'RequestVehicleProfileTarballReset',
    }, params)));
  }

  cancelVehicleProfileTarballReset(vehicleId, profileId) {
    const params = {
      vehicleId,
    };
    if (!_.isNil(profileId) && !_.isEmpty(profileId)) {
      params.profileId = profileId;
    }
    this.websocket.send(JSON.stringify(Object.assign({
      type: 'CancelVehicleProfileTarballReset',
    }, params)));
  }
}
