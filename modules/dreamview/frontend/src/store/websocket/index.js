import devConfig from "store/config/dev.yml";
import PARAMETERS from "store/config/parameters.yml";

import OfflinePlaybackWebSocketEndpoint from "store/websocket/websocket_offline";
import RealtimeWebSocketEndpoint from "store/websocket/websocket_realtime";
import MapDataWebSocketEndpoint from "store/websocket/websocket_map";

// Returns the websocket server address based on the web server address.
// Follows the convention that the websocket is served on the same host
// as the web server, the port number of websocket is the port number of
// the webserver plus one.
function deduceWebsocketServerAddr(type) {
  const server = window.location.origin;
  const link = document.createElement("a");
  link.href = server;
  const protocol = location.protocol === "https:" ? "wss" : "ws";

  let path = "";
  switch (type) {
    case "map":
      path = "map";
      break;
    case "sim_world":
      path = OFFLINE_PLAYBACK ? "RosPlayBack" : "websocket";
      break;
  }
  return `${protocol}://${link.hostname}:${window.location.port}/${path}`;
}

// NOTE: process.env.NODE_ENV will be set to "production" by webpack when
// invoked in production mode ("-p"). We rely on this to determine which
// websocket server to use.
const simWorldServerAddr =
  process.env.NODE_ENV === "production"
    ? deduceWebsocketServerAddr("sim_world")
    : `ws://${devConfig.simWorldWebsocketServer}`;
const WS = OFFLINE_PLAYBACK
  ? new OfflinePlaybackWebSocketEndpoint(simWorldServerAddr)
  : new RealtimeWebSocketEndpoint(simWorldServerAddr);
export default WS;

const mapServerAddr =
  process.env.NODE_ENV === "production"
    ? deduceWebsocketServerAddr("map")
    : `ws://${devConfig.mapWebsocketServer}`;
export const MAP_WS = new MapDataWebSocketEndpoint(mapServerAddr);
