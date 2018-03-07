import React from "react";

import PARAMETERS from "store/config/parameters.yml";
import WindowResizeControl from "components/Navigation/WindowResizeControl";
import MAP_NAVIGATOR from "components/Navigation/MapNavigator";
import BaiduMapAdapter from "components/Navigation/BaiduMapAdapter";
import GoogleMapAdapter from "components/Navigation/GoogleMapAdapter";
import WS from "store/websocket";

export default class Navigation extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            routingView: false,
        };

        this.onClickHandler = this.onClickHandler.bind(this);
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
        const mapAdapter =
            PARAMETERS.navigation.map === "GoogleMap"
                ? new GoogleMapAdapter()
                : new BaiduMapAdapter();
        MAP_NAVIGATOR.initialize(WS, mapAdapter);
        MAP_NAVIGATOR.disableControls();
    }

    componentWillUnmount() {
        MAP_NAVIGATOR.reset();
    }

    render() {
        const { viewHeight, viewWidth } = this.props;

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
