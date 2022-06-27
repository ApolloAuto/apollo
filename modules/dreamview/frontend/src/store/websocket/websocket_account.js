import STORE from 'store';
import Worker from 'utils/webworker.js';

export default class AccountWebSocketEndpoint {
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
        case 'AccountInfo':
          STORE.account.update(message.data);
          break;
        default:
          console.warn('Account WebSocket received unknown message:', message);
          break;
      }
    };
  }

  refreshAccountInfo() {
    // Refresh account info
    this.websocket.send(JSON.stringify({
      type: 'AccountInfo'
    }));
  }

  requestLogoutAccount() {
    // Logout
    this.websocket.send(JSON.stringify({
      type: 'AccountLogout',
    }));
  }
}
