import STORE from "store";
import Worker from 'utils/webworker.js';

export default class TeleopWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.worker = new Worker();
    }

    initialize() {
        try {
            this.websocket = new WebSocket(this.serverAddr);
            this.websocket.binaryType = "arraybuffer";
        } catch (error) {
            console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }

        this.websocket.onmessage = event => {
            this.worker.postMessage({
                source: 'teleop',
                data: event.data,
            });
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed with code: " + event.code);
            this.initialize();
        };

        this.worker.onmessage = event => {
            const message = event.data;
            STORE.teleop.update(message);
        };

        // Request status every 100ms.
        clearInterval(this.timer);
        this.timer = setInterval(() => {
            if (this.websocket.readyState === this.websocket.OPEN) {
                this.websocket.send(JSON.stringify({
                    type: "RequestTeleopStatus"
                }));
            }
        }, 200);
    }

    close() {
        clearInterval(this.timer);
        if (this.websocket) {
            this.websocket.onclose = undefined;
            this.websocket.close();
        }
    }

    executeCommand(command) {
        this.websocket.send(JSON.stringify({
            type: command,
        }));
    }
}