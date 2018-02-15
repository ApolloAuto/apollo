import PARAMETERS from "store/config/parameters.yml";

import GMAP_NAVIGATOR from "utils/navigation/gmap_navigator";
import NavigationWebSocketEndpoint from "store/websocket/websocket_navigation";

function deduceWebsocketServerAddr() {
    const server = window.location.origin;
    const link = document.createElement("a");
    link.href = server;
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    return `${protocol}://${link.hostname}:${PARAMETERS.server.port}/websocket`;
}

window.onload = function() {
    const serverAddr = deduceWebsocketServerAddr();
    const WS = new NavigationWebSocketEndpoint(serverAddr);
    WS.initialize();
    GMAP_NAVIGATOR.initialize(WS);
};
