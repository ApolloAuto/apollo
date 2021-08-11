import STORE from 'store';
import RENDERER from 'renderer';
import Worker from 'utils/webworker.js';

export default class MapDataWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.currentMode = null;
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
        source: 'map',
        data: event.data,
      });
    };
    this.worker.onmessage = (event) => {
      const removeOldMap =
                STORE.hmi.inNavigationMode || this.currentMode !== STORE.hmi.currentMode;
      this.currentMode = STORE.hmi.currentMode;
      RENDERER.updateMap(event.data, removeOldMap);
      STORE.setInitializationStatus(true);
    };
    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };
  }

  requestMapData(elements) {
    this.websocket.send(JSON.stringify({
      type: 'RetrieveMapData',
      elements,
    }));
  }

  requestRelativeMapData(elements) {
    this.websocket.send(JSON.stringify({
      type: 'RetrieveRelativeMapData',
      elements,
    }));
  }
}
