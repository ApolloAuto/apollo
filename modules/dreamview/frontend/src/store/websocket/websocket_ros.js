import STORE from "store";
import RENDERER from "renderer";

const protobuf = require("protobufjs/light");
const root = protobuf.Root.fromJSON(require("../../../proto_bundle/proto_bundle.json"));
const SimWorldMessage = root.lookupType("apollo.dreamview.SimulationWorld");

export default class RosWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.counter = 0;
        this.lastUpdateTimestamp = 0;
        this.lastSeqNum = -1;
        this.currMapRadius = null;
        this.updatePOI = true;
        this.updateGround = true;
    }

    initialize() {
        this.counter = 0;
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
            let message = null;
            if (typeof event.data === "string") {
                message = JSON.parse(event.data);
            } else {
                message = SimWorldMessage.toObject(
                        SimWorldMessage.decode(new Uint8Array(event.data)),
                        {enums: String});
                message.type = "SimWorldUpdate";
            }

            switch (message.type) {
                case "HMIConfig":
                    STORE.hmi.initialize(message.data);
                    break;
                case "HMIStatus":
                    STORE.hmi.updateStatus(message.data);
                    if (this.updateGround) {
                        RENDERER.updateGroundImage(STORE.hmi.currentMap);
                        this.updateGround = false;
                    }
                    break;
                case "SimWorldUpdate":
                    this.checkMessage(message);

                    STORE.updateTimestamp(message.timestamp);
                    STORE.updateModuleDelay(message);
                    RENDERER.maybeInitializeOffest(
                        message.autoDrivingCar.positionX,
                        message.autoDrivingCar.positionY);
                    STORE.meters.update(message);
                    STORE.monitor.update(message);
                    STORE.trafficSignal.update(message);
                    STORE.hmi.update(message);
                    RENDERER.updateWorld(message);
                    if (STORE.options.showPNCMonitor) {
                        STORE.planningData.update(message);
                        STORE.controlData.update(message);
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
                case "MapElementIds":
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
                if (this.updatePOI) {
                    this.requestDefaultRoutingEndPoint();
                    this.updatePOI = false;
                }

                const requestPlanningData = STORE.options.showPNCMonitor;
                this.websocket.send(JSON.stringify({
                    type : "RequestSimulationWorld",
                    planning : requestPlanningData,
                }));
            }
        }, 100);
    }

    checkMessage(world) {
        if (this.lastUpdateTimestamp !== 0
            && world.timestamp - this.lastUpdateTimestamp > 150) {
            console.log("Last sim_world_update took " +
                (world.timestamp - this.lastUpdateTimestamp) + "ms");
        }
        this.lastUpdateTimestamp = world.timestamp;
        if (this.lastSeqNum !== -1
            && world.sequenceNum > this.lastSeqNum + 1) {
            console.debug("Last seq: " + this.lastSeqNum +
                ". New seq: " + world.sequenceNum + ".");
        }
        this.lastSeqNum = world.sequenceNum;
    }

    requestMapData(elements) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapData",
            elements: elements,
        }));
    }

    requestMapElementIdsByRadius(radius) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapElementIdsByRadius",
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
        this.updatePOI = true;
        this.updateGround = true;
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
            command: command,
        }));
    }

    executeModuleCommand(module, command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteModuleCommand",
            module: module,
            command: command,
        }));
    }

    executeToolCommand(tool, command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteToolCommand",
            tool: tool,
            command: command,
        }));
    }

    changeDrivingMode(mode) {
        this.websocket.send(JSON.stringify({
            type: "ChangeDrivingMode",
            new_mode: mode,
        }));
    }

    submitDriveEvent(event_time_ms, event_msg) {
    	this.websocket.send(JSON.stringify({
            type: "SubmitDriveEvent",
            event_time_ms: event_time_ms,
            event_msg: event_msg,
        }));
    }

    toggleSimControl(enable) {
        this.websocket.send(JSON.stringify({
            type: "ToggleSimControl",
            enable: enable,
        }));
    }
}
