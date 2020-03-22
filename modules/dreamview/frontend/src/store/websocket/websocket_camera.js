import STORE from "store";
import RENDERER from "renderer";
import Worker from 'utils/webworker.js';

export default class CameraDataWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.cameraDataUpdatePeriodMs = 100;
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
                source: 'camera',
                data: event.data,
            });
        };
        this.worker.onmessage = event => {
            const message = event.data;
            switch (message.type) {
                case "CameraData":
                    if (message) {
                        STORE.cameraData.init(message, RENDERER.coordinates);
                    }
                    break;
                default:
                    console.warn('Camera WebSocket received unknown message:', message);
                    break;
            }
        };
        this.websocket.onclose = event => {
            console.log("Camera WebSocket connection closed with code: " + event.code);
            this.initialize();
        };
        // Request camera data every 100ms.
        clearInterval(this.timer);
        this.timer = setInterval(() => {
            if (this.websocket.readyState === this.websocket.OPEN
                && STORE.options.showCameraView) {
                    this.requestCameraData();
            }
        }, this.cameraDataUpdatePeriodMs);
    }

    requestCameraData() {
        this.websocket.send(JSON.stringify({
            type: "RequestCameraData",
        }));
    }
}
