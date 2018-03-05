import PARAMETERS from "store/config/parameters.yml";

import MAP_NAVIGATOR from "components/Navigation/MapNavigator";
import BaiduMapAdapter from "components/Navigation/BaiduMapAdapter";
import GoogleMapAdapter from "components/Navigation/GoogleMapAdapter";
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

    const mapAdapter =
        PARAMETERS.navigation.map === "GoogleMap" ? new GoogleMapAdapter() : new BaiduMapAdapter();
    MAP_NAVIGATOR.initialize(WS, mapAdapter);
};
