import React from "react";
import { inject, observer } from "mobx-react";

import Header from "components/Header";
import MainView from "components/Layouts/MainView";
import ToolView from "components/Layouts/ToolView";
import WS from "store/websocket";


@inject("store") @observer
export default class Dreamview extends React.Component {
    parseQueryString(queryString) {
        const params = {};

        queryString.replace('?','').split("&").forEach((query) => {
            const segments = query.split('=');
            params[segments[0]] = segments[1];
        });
        return params;
    }

    componentWillMount() {
        this.props.store.updateDimension();
    }

    componentDidMount() {
        const params = this.parseQueryString(window.location.search);
        WS.initialize(params);
        window.addEventListener("resize", () => {
            this.props.store.updateDimension();
        });
    }

    render() {
        const { isInitialized, dimension, sceneDimension, options, hmi } = this.props.store;

        return (
            <div>
                <Header />
                <div className="pane-container">
                    <div className="left-pane">
                        <div className="dreamview-body">
                            <MainView />
                            <ToolView />
                        </div>
                    </div>
                </div>
            </div>
        );
    }
}