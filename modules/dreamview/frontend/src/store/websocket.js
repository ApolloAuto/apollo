import devConfig from "store/config/dev.yml";
import STORE from "store";
import RENDERER from "renderer";

class WebSocketEndpoint {
    constructor(server) {
        this.server = server;
        this.websocket = null;
        this.counter = 0;
        this.lastUpdateTimestamp = 0;
    }

    initialize() {
        this.counter = 0;
        try {
            this.websocket = new WebSocket(this.server);
        } catch (error) {
            console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }
        this.websocket.onmessage = event => {
            const message = JSON.parse(event.data);
            switch (message.type) {
                case "sim_world_update":
                    if (this.lastUpdateTimestamp !== 0
                        && message.timestamp - this.lastUpdateTimestamp > 150) {
                        console.log("Last sim_world_update took " +
                            (message.timestamp - this.lastUpdateTimestamp) + "ms");
                    }
                    this.lastUpdateTimestamp = message.timestamp;

                    STORE.updateTimestamp(message.timestamp);
                    RENDERER.maybeInitializeOffest(
                        message.world.autoDrivingCar.positionX,
                        message.world.autoDrivingCar.positionY);
                    RENDERER.updateWorld(message.world);
                    STORE.meters.update(message.world);
                    STORE.monitor.update(message.world);
                    if (message.mapHash && (this.counter % 10 === 0)) {
                        // NOTE: This is a hack to limit the rate of map updates.
                        this.counter = 0;
                        RENDERER.updateMapIndex(message.mapHash, message.mapElementIds);
                    }
                    this.counter += 1;
                    break;
                case "MapData":
                    RENDERER.updateMap(message.data);
                    break;
            }
        };
        this.websocket.onclose = event => {
            console.log(event);
            this.initialize();
        };
    }

    requestMapData(elements) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapData",
            elements: elements,
        }));
    }
}

// Returns the websocket server address based on the web server address.
// Follows the convention that the websocket is served on the same host
// as the web server, the port number of websocket is the port number of
// the webserver plus one.
function deduceWebsocketServer() {
    const server = window.location.origin;
    const link = document.createElement("a");
    link.href = server;
    return `ws://${link.hostname}:${window.location.port}/websocket`;
}

// NOTE: process.env.NODE_ENV will be set to "production" by webpack when
// invoked in production mode ("-p"). We rely on this to determine which
// websocket server to use.
const server = process.env.NODE_ENV === "production" ?
               deduceWebsocketServer() : `ws://${devConfig.websocketServer}`;
const WS = new WebSocketEndpoint(server);

export default WS;
