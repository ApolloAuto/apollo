import devConfig from "store/config/dev.yml";
import STORE from "store";
import RENDERER from "renderer";

export default class PlaybackWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.lastUpdateTimestamp = 0;
        this.lastSeqNum = -1;

        // For JSON playback mode
        this.params = null;
        this.numFrames = null;
        this.currentFrameId = null;
    }

    initialize(params) {
        if (params) {
            // TODO (vlin): validate all required parameters.
            this.params = params;
            if (params.url) {
                this.params.serverUrl = `${location.protocol}//${params.url}`;
            }
        }

        try {
            this.websocket = new WebSocket(this.serverAddr);
        } catch (error) {
            console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }
        this.websocket.onopen = event => {
            if (this.params && this.params.mapId && this.params.id) {
                this.websocket.send(JSON.stringify({
                    type: 'RetrieveGroundMeta',
                    data: {
                        mapId: this.params.mapId,
                    }
                }));
            }
        };
        this.websocket.onmessage = event => {
            const message = JSON.parse(event.data);

            switch (message.type) {
                case "GroundMetadata":
                    RENDERER.updateGroundMetadata(this.params.serverUrl, message.data);
                    this.requstFrameCount(this.params.id);
                    break;
                case "FrameCount":
                    this.numFrames = message.data;
                    this.currentFrameId = 1;
                    STORE.setInitializationStatus(true);
                    break;
                case "SimWorldUpdate":
                    this.checkMessage(message);

                    STORE.updateTimestamp(message.timestamp);
                    STORE.updateWorldTimestamp(message.world.timestampSec);
                    RENDERER.maybeInitializeOffest(
                        message.world.autoDrivingCar.positionX,
                        message.world.autoDrivingCar.positionY);
                    RENDERER.updateWorld(message.world, message.planningData);
                    STORE.meters.update(message.world);
                    STORE.monitor.update(message.world);
                    STORE.trafficSignal.update(message.world);
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
            if (this.websocket.readyState === this.websocket.OPEN && this.numFrames) {
                this.websocket.send(JSON.stringify({
                    type : "RequestSimulationWorld",
                    jobId: this.params.id,
                    frameId: this.currentFrameId,
                }));

                this.currentFrameId ++;
                if (this.currentFrameId > this.numFrames) {
                    clearInterval(this.timer);
                }
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

    requstFrameCount(id) {
        this.websocket.send(JSON.stringify({
            type: 'RetrieveFrameCount',
            id: id,
        }));
    }

    requstFrameData(id) {
        this.websocket.send(JSON.stringify({
            type: 'RetrieveFrameData',
            data: {
                id: id,
            }
        }));
    }
}