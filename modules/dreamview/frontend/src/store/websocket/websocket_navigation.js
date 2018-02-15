import Worker from 'worker-loader!utils/webworker.js';
import GMAP_NAVIGATOR from "utils/navigation/gmap_navigator";

export default class NavigationWebSocketEndpoint {
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
                source: 'realtime',
                data: event.data,
            });
        };
        this.worker.onmessage = event => {
            const message = event.data;
            switch (message.type) {
                case "SimWorldUpdate":
                    GMAP_NAVIGATOR.update(message);
                    break;
                default:
                    break;
            }
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed, close_code: " + event.code);
            this.initialize();
        };

        // Request simulation world every 100ms.
        clearInterval(this.timer);
        this.timer = setInterval(() => {
            if (this.websocket.readyState === this.websocket.OPEN) {
                this.requestSimulationWorld();
            }
        }, 100);
    }

    requestSimulationWorld() {
        this.websocket.send(JSON.stringify({
            type : "RequestSimulationWorld",
            planning : false,
        }));
    }

    publishNavigationInfo(data) {
        this.websocket.send(data);
    }
}
