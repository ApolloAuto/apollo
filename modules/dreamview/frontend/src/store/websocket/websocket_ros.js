import STORE from "store";
import RENDERER from "renderer";

export default class RosWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.counter = 0;
        this.lastUpdateTimestamp = 0;
        this.lastSeqNum = -1;
        this.currMapRadius = null;
        this.updatePOI = true;
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
                    const world = JSON.parse(message.world);
                    this.checkMessage(message, world);

                    STORE.updateTimestamp(message.timestamp);
                    STORE.updateWorldTimestamp(world.timestampSec);
                    STORE.updateModuleDelay(world);
                    RENDERER.maybeInitializeOffest(
                        world.autoDrivingCar.positionX,
                        world.autoDrivingCar.positionY);
                    RENDERER.updateWorld(world, message.planningData);
                    STORE.meters.update(world);
                    STORE.monitor.update(world);
                    STORE.trafficSignal.update(world);
                    STORE.hmi.update(world);
                    if (STORE.options.showPNCMonitor) {
                        STORE.planning.update(world, message.planningData);
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

    checkMessage(message, world) {
        if (this.lastUpdateTimestamp !== 0
            && message.timestamp - this.lastUpdateTimestamp > 150) {
            console.log("Last sim_world_update took " +
                (message.timestamp - this.lastUpdateTimestamp) + "ms");
        }
        this.lastUpdateTimestamp = message.timestamp;
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
        this.updatePOI = true;
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
