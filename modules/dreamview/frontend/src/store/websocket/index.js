import devConfig from "store/config/dev.yml";
import PARAMETERS from "store/config/parameters.yml";

import OfflinePlaybackWebSocketEndpoint from "store/websocket/websocket_offline";
import RealtimeWebSocketEndpoint from "store/websocket/websocket_ros";


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

const WS = PARAMETERS.offlinePlayback
            ? new OfflinePlaybackWebSocketEndpoint(serverAddr)
            : new RealtimeWebSocketEndpoint(serverAddr);

export default WS;

