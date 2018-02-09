import STORE from "store";
import RENDERER from "renderer";

const protobuf = require("protobufjs/light");
const root = protobuf.Root.fromJSON(require("../../../proto_bundle/map_proto_bundle.json"));
const mapMessage = root.lookupType("apollo.hdmap.Map");

export default class MapDataWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
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
            const data = mapMessage.toObject(
                    mapMessage.decode(new Uint8Array(event.data)), {enums: String});

            RENDERER.updateMap(data);
            STORE.setInitializationStatus(true);
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed, close_code: " + event.code);
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
