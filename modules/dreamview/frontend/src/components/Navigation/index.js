import React from "react";

import PARAMETERS from "store/config/parameters.yml";
import WindowResizeControl from "components/Navigation/WindowResizeControl";
import MAP_NAVIGATOR from "components/Navigation/MapNavigator";
import WS from "store/websocket";
import loadScriptAsync from "utils/script_loader";

export default class Navigation extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            routingView: false,
        };

        this.onClickHandler = this.onClickHandler.bind(this);
        this.scriptOnLoadHandler = this.scriptOnLoadHandler.bind(this);
    }

    onClickHandler() {
        const newRoutingView = !this.state.routingView;
        if (newRoutingView) {
            MAP_NAVIGATOR.enableControls();
        } else {
            MAP_NAVIGATOR.disableControls();
        }
        this.setState({
            routingView: newRoutingView,
        });
    }

    componentDidMount() {
        if (MAP_NAVIGATOR.mapAPILoaded) {
            this.scriptOnLoadHandler();
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
        const { viewHeight, viewWidth } = this.props;

        if (PARAMETERS.navigation.map !== "GoogleMap" && PARAMETERS.navigation.map !== "BaiduMap") {
            console.error(`Map API ${PARAMETERS.navigation.map} is not supported.`);
            return null;
        }

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

        let top = 0;
        let left = 0;
        let width = viewWidth;
        let height = viewHeight;
        let iconType = "maximizing";
        if (!this.state.routingView) {
            top = 10;
            left = 20;
            width = Math.min(viewWidth * 0.3, 250);
            height = Math.min(viewHeight * 0.5, 300);
            iconType = "minimizing";
        }

        return (
            <div
                displayname="navigation"
                className="navigation-view"
                style={{ width: width, height: height, top: top, left: left }} >
                <div id="map_canvas" />
                <WindowResizeControl type={iconType} onClick={this.onClickHandler} />
            </div>
        );
    }
}
