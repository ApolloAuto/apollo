import devConfig from "store/config/dev.yml";
import STORE from "store";
import RENDERER from "renderer";


export default class OfflinePlaybackWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.lastUpdateTimestamp = 0;
        this.lastSeqNum = -1;
        this.requestTimer = null;
        this.processTimer = null;
        this.frameData = {}; // cache frames
    }

    initialize(params) {
        if (params && params.url && params.id && params.map) {
            this.serverUrl = `${location.protocol}//${params.url}`;
            STORE.playback.setJobId(params.id);
            STORE.playback.setMapId(params.map);
        } else {
            console.error("ERROR: missing required parameter(s)");
            return;
        }

        try {
            this.websocket = new WebSocket(this.serverAddr);
        } catch (error) {
            console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize(params);
            }, 1000);
            return;
        }
        this.websocket.onopen = event => {
            this.requestGroundMeta(STORE.playback.mapId);
        };
        this.websocket.onmessage = event => {
            const message = JSON.parse(event.data);

            switch (message.type) {
                case "GroundMetadata":
                    RENDERER.updateGroundMetadata(this.serverUrl, message.data);
                    this.requstFrameCount(STORE.playback.jobId);
                    break;
                case "FrameCount":
                    STORE.playback.setNumFrames(message.data);
                    this.requestSimulationWorld(STORE.playback.jobId, STORE.playback.next());
                    break;
                case "SimWorldUpdate":
                    this.checkMessage(message);
                    STORE.setInitializationStatus(true);

                    if (STORE.playback.isSeeking) {
                        this.processSimWorld(message);
                    }

                    if (message.timestamp && !(message.timestamp in this.frameData)) {
                        this.frameData[message.timestamp] = message;
                    }

                    break;
            }
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed, close_code: " + event.code);
            this.initialize(params);
        };
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

    startPlayback(msPerFrame) {
        clearInterval(this.requestTimer);
        this.requestTimer = setInterval(() => {
            if (this.websocket.readyState === this.websocket.OPEN && STORE.playback.initialized()) {
                this.requestSimulationWorld(STORE.playback.jobId, STORE.playback.next());

                if (!STORE.playback.hasNext()) {
                    clearInterval(this.requestTimer);
                    this.requestTimer = null;
                }
            }
        }, msPerFrame);

        clearInterval(this.processTimer);
        this.processTimer = setInterval(() => {
            if (STORE.playback.initialized()) {
                const timestamp = STORE.playback.seekingFrame * 100;
                if (timestamp in this.frameData) {
                    this.processSimWorld(this.frameData[timestamp]);
                }

                if (STORE.playback.replayComplete) {
                    clearInterval(this.processTimer);
                    this.processTimer = null;
                }
            }
        }, msPerFrame);
    }

    pausePlayback() {
        clearInterval(this.requestTimer);
        clearInterval(this.processTimer);
        this.requestTimer = null;
        this.processTimer = null;
    }

    requestGroundMeta(mapId) {
        this.websocket.send(JSON.stringify({
            type: 'RetrieveGroundMeta',
            mapId: mapId,
        }));
    }

    processSimWorld(message) {
        if (STORE.playback.shouldProcessFrame(message.world)) {
            STORE.updateTimestamp(message.timestamp);
            STORE.updateWorldTimestamp(message.world.timestampSec);
            RENDERER.maybeInitializeOffest(
                message.world.autoDrivingCar.positionX,
                message.world.autoDrivingCar.positionY);
            RENDERER.updateWorld(message.world, message.planningData);
            STORE.meters.update(message.world);
            STORE.monitor.update(message.world);
            STORE.trafficSignal.update(message.world);
        }
    }
    requstFrameCount(jobId) {
        this.websocket.send(JSON.stringify({
            type: 'RetrieveFrameCount',
            jobId: jobId,
        }));
    }

    requestSimulationWorld(jobId, frameId) {
        const timestamp = frameId * 100;
        if (!(timestamp in this.frameData)) {
            this.websocket.send(JSON.stringify({
                type : "RequestSimulationWorld",
                jobId: jobId,
                frameId: frameId,
            }));
        } else if (STORE.playback.isSeeking) {
            this.processSimWorld(this.frameData[timestamp]);
        }
    }
}