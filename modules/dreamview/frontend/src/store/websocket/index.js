import devConfig from "store/config/dev.yml";
import PARAMETERS from "store/config/parameters.yml";

import JsonWebSocketEndpoint from "store/websocket/websocket_json";
import RosWebSocketEndpoint from "store/websocket/websocket_ros";


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

let WS = null;
switch (PARAMETERS.websocket.type) {
    case 'json':
        WS = new JsonWebSocketEndpoint(serverAddr);
        break;
    case 'ros':
    default:
        console.log('WS:ros');
        WS = new RosWebSocketEndpoint(serverAddr);
        break;
}

export default WS;

