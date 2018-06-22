import PARAMETERS from "store/config/parameters.yml";

import OfflinePlaybackWebSocketEndpoint from "store/websocket/websocket_offline";
import RealtimeWebSocketEndpoint from "store/websocket/websocket_realtime";
import MapDataWebSocketEndpoint from "store/websocket/websocket_map";
import PointCloudWebSocketEndpoint from "store/websocket/websocket_point_cloud";

// Returns the websocket server address based on the web server address.
// Follows the convention that the websocket is served on the same host/port
// as the web server.
function deduceWebsocketServerAddr(type) {
    const server = window.location.origin;
    const link = document.createElement("a");
    link.href = server;
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    const port =
        process.env.NODE_ENV === "production" ? window.location.port : PARAMETERS.server.port;

    let path = "";
    switch (type) {
        case "map":
            path = "map";
            break;
        case "point_cloud":
            path = "pointcloud";
            break;
        case "sim_world":
            path = OFFLINE_PLAYBACK ? "RosPlayBack" : "websocket";
            break;
    }
    return `${protocol}://${link.hostname}:${port}/${path}`;
}

// NOTE: process.env.NODE_ENV will be set to "production" by webpack when
// invoked in production mode ("-p"). We rely on this to determine which
// websocket server to use.
const simWorldServerAddr = deduceWebsocketServerAddr("sim_world");
const WS = OFFLINE_PLAYBACK
    ? new OfflinePlaybackWebSocketEndpoint(simWorldServerAddr)
    : new RealtimeWebSocketEndpoint(simWorldServerAddr);
export default WS;

const mapServerAddr = deduceWebsocketServerAddr("map");
export const MAP_WS = new MapDataWebSocketEndpoint(mapServerAddr);

const pointCloudServerAddr = deduceWebsocketServerAddr("point_cloud");
export const POINT_CLOUD_WS = new PointCloudWebSocketEndpoint(pointCloudServerAddr);
