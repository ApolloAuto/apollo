import STORE from "store";
import RENDERER from "renderer";
import Worker from 'utils/webworker.js';

export default class PointCloudWebSocketEndpoint {
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
                source: 'point_cloud',
                data: event.data,
            });
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed with code: " + event.code);
            this.initialize();
        };
        this.worker.onmessage = event => {
            if (event.data.type === "PointCloudStatus") {
              STORE.setOptionStatus('showPointCloud', event.data.enabled);
              if (STORE.options.showPointCloud === false) {
                RENDERER.updatePointCloud({num:[]});
              }
            } else if (STORE.options.showPointCloud === true && event.data.num !== undefined) {
                RENDERER.updatePointCloud(event.data);
            }
        };
        // Request point cloud every 100ms.
        clearInterval(this.timer);
        this.timer = setInterval(() => {
            if (this.websocket.readyState === this.websocket.OPEN
                && STORE.options.showPointCloud === true) {
                this.websocket.send(JSON.stringify({
                    type : "RequestPointCloud"
                }));
            }
        }, 100);
    }

    togglePointCloud(enable) {
        this.websocket.send(JSON.stringify({
            type: "TogglePointCloud",
            enable: enable,
        }));
        if (STORE.options.showPointCloud === false) {
            RENDERER.updatePointCloud({num:[]});
        }
    }
}