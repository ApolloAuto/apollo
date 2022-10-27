import STORE from 'store';
import Worker from 'utils/webworker.js';

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
    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'point_cloud',
        data: event.data,
      });
    };
    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };
    this.worker.onmessage = (event) => {
      if(event.data.type === 'PluginMsg') {
        const message = event.data;

        switch (message.data.name) {
          case 'StudioConnectorCertStatus':
            const status = JSON.parse(message.data.info).status;
            if (status === 'OK') {
              this.getScenarioSetList();
            }
            STORE.studioConnector.checkCertificate(status);
            break;
          // Get Scenario Set Info Success
          case 'GetScenarioSetListSuccess':
            STORE.studioConnector.updateRemoteScenarioSetList(
              JSON.parse(message.data.info));
            break;
          // Get Scenario Set Info Failed
          case 'GetScenarioSetListFail':
            STORE.studioConnector.updateRemoteScenarioSetList(
              JSON.parse(message.data.info)
            );
            break;
          case 'DownloadScenarioSetSuccess':
            STORE.studioConnector.updateRemoteScenarioSetStatus(
              JSON.parse(message.data.info)?.scenario_set_id,
              JSON.parse(message.data.info)?.status,
            );
            break;
          case 'DownloadScenarioSetFail':
            STORE.studioConnector.updateRemoteScenarioSetStatus(
              JSON.parse(message.data.info)?.scenario_set_id,
              'fail',
              JSON.parse(message.data.info)?.error_msg,
            );
            break;
        }
      }
    };
    return this;
  }

  checkCertificate() {
    this.websocket.send(JSON.stringify({
      type: 'PluginRequest',
      data: {
        'name': 'CheckCertStatus',
        'source': 'dreamview',
        'info': '',
        'target': 'studio_connector',
        'source_type': 'module',
        'target_type': 'plugins',
      }
    }));
    return this;
  }


  getScenarioSetList() {
    this.websocket.send(JSON.stringify({
      type: 'PluginRequest',
      data: {
        'name': 'GetScenarioSetList',
        'source': 'dreamview',
        'info': '',
        'target': 'studio_connector',
        'source_type': 'module',
        'target_type': 'plugins',
      }
    }));
    return this;
  }

  downloadScenarioSetById(scenarioSetId) {
    this.websocket.send(JSON.stringify({
      type: 'PluginRequest',
      data: {
        'name': 'DownloadScenarioSet',
        'source': 'dreamview',
        'info': scenarioSetId,
        'target': 'studio_connector',
        'source_type': 'module',
        'target_type': 'plugins',
      }
    }));
    return this;
  }

  // 下载record
  downloadRecord() {
    this.websocket.send(JSON.stringify({
      type: 'PluginRequest',
      data: {
        'name': 'DownloadRecord',
        'source': 'dreamview',
        'info': '',
        'target': 'studio_connector',
        'source_type': 'module',
        'target_type': 'plugins',
      }
    }));
    return this;
  }


}
