import React from "react";

import MAP_NAVIGATOR from "components/Navigation/MapNavigator";
import WS from "store/websocket";
import { MAP_SIZE } from "store/dimension";
import loadScriptAsync from "utils/script_loader";

import WindowResizeControl from "components/Navigation/WindowResizeControl";

export default class Navigation extends React.Component {
    constructor(props) {
        super(props);

        this.scriptOnLoadHandler = this.scriptOnLoadHandler.bind(this);

        if (!MAP_NAVIGATOR.mapAPILoaded) {
            let onLoad = () => {
                console.log("Map API script loaded.");
            };
            if (PARAMETERS.navigation.map === "BaiduMap") {
                // For Baidu Map, the callback function is set in the window Object level
                window.initMap = this.scriptOnLoadHandler;
            } else if (PARAMETERS.navigation.map === "GoogleMap") {
                // For Google Map, the callback function is set from the <Script>
                onLoad = this.scriptOnLoadHandler;
            }

            loadScriptAsync({
                url: PARAMETERS.navigation.mapAPiUrl,
                onLoad: onLoad,
                onError: () => {
                    console.log("Failed to load map api");
                },
            });
        }
    }

    componentDidMount() {
        if (MAP_NAVIGATOR.mapAPILoaded) {
            this.scriptOnLoadHandler();
        }
    }

    componentDidUpdate() {
        const { hasRoutingControls, size } = this.props;

        if (hasRoutingControls && size === MAP_SIZE.FULL) {
            MAP_NAVIGATOR.enableControls();
        } else {
            MAP_NAVIGATOR.disableControls();
        }
    }

    scriptOnLoadHandler() {
        import(`components/Navigation/${PARAMETERS.navigation.map}Adapter`).then(
            mapAdapterModule => {
                const MapAdapterClass = mapAdapterModule.default;
                const mapAdapter = new MapAdapterClass();
                MAP_NAVIGATOR.mapAPILoaded = true;
                MAP_NAVIGATOR.initialize(WS, mapAdapter);
                MAP_NAVIGATOR.disableControls();
            }
        );
    }

    componentWillUnmount() {
        MAP_NAVIGATOR.reset();
    }

    render() {
        const { width, height, size, onResize } = this.props;

        if (!["GoogleMap", "BaiduMap"].includes(PARAMETERS.navigation.map)) {
            console.error(`Map API ${PARAMETERS.navigation.map} is not supported.`);
            return null;
        }

        return (
            <div displayname="navigation" className="navigation-view" style={{ width, height }} >
                <div id="map_canvas" />
                <WindowResizeControl type={size} onClick={onResize} />
            </div>
        );
    }
}
