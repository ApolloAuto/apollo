import React from "react";
import { inject, observer } from "mobx-react";

import SideBar from "components/SideBar";
import StatusBar from "components/StatusBar";
import Scene from "components/Scene";
import RouteEditingBar from "components/RouteEditingBar";
import Loader from "components/common/Loader";
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
        const { dimension, meters, options,
                routeEditingManager, isInitialized} = this.props.store;

        if (!isInitialized) {
            return <Loader />;
        }

        return (
            <div>
                {!routeEditingManager.inEditingView ? <SideBar /> : null}
                {!routeEditingManager.inEditingView ? <StatusBar meters={meters} /> : null}
                { routeEditingManager.inEditingView ? <RouteEditingBar /> : null}
                <Scene width={dimension.width} height={dimension.height} options={options}/>
            </div>
        );
    }
}
