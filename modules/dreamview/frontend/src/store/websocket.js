import devConfig from "store/config/dev.yml";
import STORE from "store";
import RENDERER from "renderer";

const _ = require('lodash');

class WebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.counter = 0;
        this.lastUpdateTimestamp = 0;
        this.lastSeqNum = -1;
        this.currMapRadius = null;
    }

    initialize() {
        this.counter = 0;
        try {
            this.websocket = new WebSocket(this.serverAddr);
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
                case "HMIConfig":
                    STORE.hmi.initialize(message.data);
                    break;
                case "HMIStatus":
                    STORE.hmi.updateStatus(message.data);
                    break;
                case "SimWorldUpdate":
                    this.checkMessage(message);

                    STORE.updateTimestamp(message.timestamp);
                    STORE.updateWorldTimestamp(message.world.timestampSec);
                    RENDERER.maybeInitializeOffest(
                        message.world.autoDrivingCar.positionX,
                        message.world.autoDrivingCar.positionY);
                    RENDERER.updateWorld(message.world);
                    STORE.meters.update(message.world);
                    STORE.monitor.update(message.world);
                    if (STORE.options.showPNCMonitor) {
                        STORE.planning.update(message.world);
                    }
                    if (message.mapHash && (this.counter % 10 === 0)) {
                        // NOTE: This is a hack to limit the rate of map updates.
                        this.counter = 0;
                        this.currMapRadius = message.mapRadius;
                        RENDERER.updateMapIndex(message.mapHash,
                            message.mapElementIds, message.mapRadius);
                    }
                    this.counter += 1;
                    break;
                case "MapElements":
                    RENDERER.updateMapIndex(message.mapHash,
                            message.mapElementIds, message.mapRadius);
                    break;
                case "MapData":
                    RENDERER.updateMap(message.data);
                    STORE.setInitializationStatus(true);
                    break;
                case "DefaultEndPoint":
                    STORE.routeEditingManager.updateDefaultRoutingEndPoint(message);
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
                // Load default routing end point.
                if (_.isEmpty(STORE.routeEditingManager.defaultRoutingEndPoint)) {
                    this.requestDefaultRoutingEndPoint();
                }
                this.websocket.send(JSON.stringify({type : "RequestSimulationWorld"}));
            }
        }, 100);
    }

    checkMessage(message) {
        if (this.lastUpdateTimestamp !== 0
            && message.timestamp - this.lastUpdateTimestamp > 150) {
            console.log("Last sim_world_update took " +
                (message.timestamp - this.lastUpdateTimestamp) + "ms");
        }
        this.lastUpdateTimestamp = message.timestamp;
        if (this.lastSeqNum !== -1
            && message.world.sequenceNum > this.lastSeqNum + 1) {
            console.debug("Last seq: " + this.lastSeqNum +
                ". New seq: " + message.world.sequenceNum + ".");
        }
        this.lastSeqNum = message.world.sequenceNum;
    }

    requestMapData(elements) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapData",
            elements: elements,
        }));
    }

    requestMapElementsByRadius(radius) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapElementsByRadius",
            radius: radius,
        }));
    }

    requestRoute(start, waypoint, end) {
        this.websocket.send(JSON.stringify({
            type: "SendRoutingRequest",
            start: start,
            end: end,
            waypoint: waypoint,
        }));
    }

    requestDefaultRoutingEndPoint() {
        this.websocket.send(JSON.stringify({
            type: "GetDefaultEndPoint",
        }));
    }

    resetBackend() {
        this.websocket.send(JSON.stringify({
            type: "Reset",
        }));
    }

    dumpMessages() {
        this.websocket.send(JSON.stringify({
            type: "Dump",
        }));
    }

    changeSetupMode(mode) {
        this.websocket.send(JSON.stringify({
            type: "ChangeMode",
            new_mode: mode,
        }));
    }

    changeMap(map) {
        this.websocket.send(JSON.stringify({
            type: "ChangeMap",
            new_map: map,
        }));
    }

    changeVehicle(vehcile) {
        this.websocket.send(JSON.stringify({
            type: "ChangeVehicle",
            new_vehicle: vehcile,
        }));
    }

    executeModeCommand(command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteModeCommand",
            command, command,
        }));
    }

    executeModuleCommand(module, command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteModuleCommand",
            module: module,
            command, command,
        }));
    }

    executeToolCommand(tool, command) {
         this.websocket.send(JSON.stringify({
            type: "ExecuteToolCommand",
            tool: tool,
            command, command,
        }));
    }

    changeDrivingMode(mode) {
         this.websocket.send(JSON.stringify({
            type: "ChangeDrivingMode",
            new_mode: mode,
        }));
    }
}

// Returns the websocket server address based on the web server address.
// Follows the convention that the websocket is served on the same host
// as the web server, the port number of websocket is the port number of
// the webserver plus one.
function deduceWebsocketServerAddr() {
    const server = window.location.origin;
    const link = document.createElement("a");
    link.href = server;
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    return `${protocol}://${link.hostname}:${window.location.port}/websocket`;
}

// NOTE: process.env.NODE_ENV will be set to "production" by webpack when
// invoked in production mode ("-p"). We rely on this to determine which
// websocket server to use.
const serverAddr = process.env.NODE_ENV === "production" ?
                   deduceWebsocketServerAddr() : `ws://${devConfig.websocketServer}`;
const WS = new WebSocketEndpoint(serverAddr);

export default WS;
