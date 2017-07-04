import React from "react";
import { inject, observer } from "mobx-react";

import SideBar from "components/SideBar";
import StatusBar from "components/StatusBar";
import Scene from "components/Scene";
import WS from "store/websocket";

@inject("store") @observer
export default class Dreamview extends React.Component {
    componentDidMount() {
        WS.initialize();
        window.addEventListener("resize", () => {
            this.props.store.updateDimension();
        });
    }

    render() {
        const { dimension, meters } = this.props.store;

        return (
            <div>
                <SideBar />
                <StatusBar meters={meters} />
                <Scene width={dimension.width} height={dimension.height} />
            </div>
        );
    }
}
