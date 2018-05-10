import STORE from "store";
import RENDERER from "renderer";
import MAP_NAVIGATOR from "components/Navigation/MapNavigator";
import Worker from 'utils/webworker.js';

export default class RosWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.simWorldUpdatePeriodMs = 100;
        this.simWorldLastUpdateTimestamp = 0;
        this.mapUpdatePeriodMs = 1000;
        this.mapLastUpdateTimestamp = 0;
        this.updatePOI = true;
        this.routingTime = undefined;
        this.currentMode = null;
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
                case "HMIConfig":
                    STORE.hmi.initialize(message.data);
                    break;
                case "HMIStatus":
                    STORE.hmi.updateStatus(message.data);
                    RENDERER.updateGroundImage(STORE.hmi.currentMap);
                    break;
                case "VehicleParam":
                    STORE.hmi.updateVehicleParam(message.data);
                    break;
                case "SimControlStatus":
                    STORE.setOptionStatus('simControlEnabled', message.enabled);
                    break;
                case "SimWorldUpdate":
                    this.checkMessage(message);

                    const updateCoordination = (this.currentMode !== STORE.hmi.currentMode);
                    this.currentMode = STORE.hmi.currentMode;
                    if (STORE.hmi.inNavigationMode) {
                        // In navigation mode, relative map is set and the relative position
                        // of auto driving car is (0, 0). Absolute position of the car
                        // only needed in MAP_NAVIGATOR.
                        if (MAP_NAVIGATOR.isInitialized()) {
                            MAP_NAVIGATOR.update(message);
                        }
                        message.autoDrivingCar.positionX = 0;
                        message.autoDrivingCar.positionY = 0;
                        message.autoDrivingCar.heading = 0;

                        RENDERER.coordinates.setSystem("FLU");
                        this.mapUpdatePeriodMs = 100;
                    } else {
                        RENDERER.coordinates.setSystem("ENU");
                        this.mapUpdatePeriodMs = 1000;
                    }

                    STORE.updateTimestamp(message.timestamp);
                    STORE.updateModuleDelay(message);
                    RENDERER.maybeInitializeOffest(
                        message.autoDrivingCar.positionX,
                        message.autoDrivingCar.positionY,
                        updateCoordination);
                    STORE.meters.update(message);
                    STORE.monitor.update(message);
                    STORE.trafficSignal.update(message);
                    STORE.hmi.update(message);
                    RENDERER.updateWorld(message);
                    this.updateMapIndex(message);
                    if (STORE.options.showPNCMonitor) {
                        STORE.planningData.update(message);
                        STORE.controlData.update(message, STORE.hmi.vehicleParam);
                    }
                    if (this.routingTime !== message.routingTime) {
                        // A new routing needs to be fetched from backend.
                        this.requestRoutePath();
                        this.routingTime = message.routingTime;
                    }
                    break;
                case "MapElementIds":
                    RENDERER.updateMapIndex(message.mapHash,
                            message.mapElementIds, message.mapRadius);
                    break;
                case "DefaultEndPoint":
                    STORE.routeEditingManager.updateDefaultRoutingEndPoint(message);
                    break;
                case "RoutePath":
                    RENDERER.updateRouting(message.routingTime, message.routePath);
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
        }, this.simWorldUpdatePeriodMs);
    }

    updateMapIndex(message) {
        const now = new Date();
        const duration = now - this.mapLastUpdateTimestamp;
        if (message.mapHash && duration >= this.mapUpdatePeriodMs) {
            RENDERER.updateMapIndex(message.mapHash, message.mapElementIds, message.mapRadius);
            this.mapLastUpdateTimestamp = now;
        }
    }

    checkMessage(world) {
        const now = new Date().getTime();
        const duration = now - this.simWorldLastUpdateTimestamp;
        if (this.simWorldLastUpdateTimestamp !== 0 && duration > 250) {
            console.log("Last sim_world_update took " + duration + "ms");
        }
        this.simWorldLastUpdateTimestamp = now;
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

    submitDriveEvent(eventTimeMs, eventMessage) {
    	this.websocket.send(JSON.stringify({
            type: "SubmitDriveEvent",
            event_time_ms: eventTimeMs,
            event_msg: eventMessage,
        }));
    }

    sendVoicePiece(data) {
        this.websocket.send(JSON.stringify({
            type: "VoicePiece",
            data: btoa(String.fromCharCode(...data)),
        }));
    }

    toggleSimControl(enable) {
        this.websocket.send(JSON.stringify({
            type: "ToggleSimControl",
            enable: enable,
        }));
    }

    requestRoutePath() {
        this.websocket.send(JSON.stringify({
            type: "RequestRoutePath",
        }));
    }

    publishNavigationInfo(data) {
        this.websocket.send(data);
    }
}
