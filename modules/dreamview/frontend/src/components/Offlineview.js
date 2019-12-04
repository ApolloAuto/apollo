import React from "react";
import { inject, observer } from "mobx-react";

import MainView from "components/Layouts/MainView";
import ToolView from "components/Layouts/ToolView";
import Loader from "components/common/Loader";

import HOTKEYS_CONFIG from "store/config/hotkeys.yml";
import WS from "store/websocket";

@inject("store") @observer
export default class Offlineview extends React.Component {
    constructor(props) {
        super(props);

        this.handleKeyPress = this.handleKeyPress.bind(this);
        this.updateDimension = this.props.store.dimension.update.bind(this.props.store.dimension);
    }

    parseQueryString(queryString) {
        const params = {};

        queryString.replace('?','').split("&").forEach((query) => {
            const segments = query.split('=');
            params[segments[0]] = segments[1];
        });
        return params;
    }

    componentWillMount() {
        this.updateDimension();
    }

    componentDidMount() {
        const params = this.parseQueryString(window.location.search);
        WS.initialize(params);
        window.addEventListener("resize", this.updateDimension, false);
        window.addEventListener("keypress", this.handleKeyPress, false);
    }

    componentWillUnmount() {
        window.removeEventListener("resize", this.updateDimension, false);
        window.removeEventListener("keypress", this.handleKeyPress, false);
    }

    handleKeyPress(event) {
        const optionName = HOTKEYS_CONFIG[event.key];
        if (!optionName) {
            return;
        }

        const { options } = this.props.store;
        event.preventDefault();
        if (optionName === "cameraAngle") {
            options.rotateCameraAngle();
        }
    }

    render() {
        const { isInitialized, offlineViewErrorMsg } = this.props.store;

        if (!isInitialized) {
            return (
                <div className="offlineview">
                    <Loader extraClasses="offline-loader" offlineViewErr={offlineViewErrorMsg} />
                </div>
            );
        }

        return (
            <div className="offlineview">
                <MainView />
                <ToolView />
            </div>
        );
    }
}