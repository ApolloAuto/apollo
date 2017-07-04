import devConfig from "store/config/dev.yml";
import STORE from "store";
import RENDERER from "renderer";

class WebSocketEndpoint {
    constructor(server) {
        this.server = server;
        this.websocket = null;
    }

    initialize() {
        try {
            this.websocket = new WebSocket(this.server);
        } catch (error) {
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }
        this.websocket.onmessage = event => {
            const message = JSON.parse(event.data);
            switch (message.type) {
                case "sim_world_update":
                    STORE.updateTimestamp(message.timestamp);
                    RENDERER.maybeInitializeOffest(
                        message.world.autoDrivingCar.positionX,
                        message.world.autoDrivingCar.positionY);
                    RENDERER.updateWorld(message.world);
                    STORE.meters.update(message.world);
                    STORE.monitor.update(message.world);
                    break;
            }
        };
        this.websocket.onclose = () => {
            setTimeout(() => {
                this.initialize();
            }, 1000);
        };
    }
}

// Returns the websocket server address based on the web server address.
// Follows the convention that the websocket is served on the same host
// as the web server, the port number of websocket is the port number of
// the webserver plus one.
function deduceWebsocketServer() {
    const server = window.location.origin;
    const websocket_port = parseInt(window.location.port) + 1;
    const link = document.createElement("a");
    link.href = server;
    return `ws://${link.hostname}:${websocket_port}`;
}

// NOTE: process.env.NODE_ENV will be set to "production" by webpack when
// invoked in production mode ("-p"). We rely on this to determine which
// websocket server to use.
const server = process.env.NODE_ENV === "production" ?
               deduceWebsocketServer() : `ws://${devConfig.websocketServer}`;
const WS = new WebSocketEndpoint(server);

export default WS;
