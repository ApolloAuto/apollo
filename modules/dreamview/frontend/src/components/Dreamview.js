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

        const showBars = !routeEditingManager.inEditingView && isInitialized;
        const showRoutingBar = routeEditingManager.inEditingView;
        const showLoader = !isInitialized;

        return (
            <div>
                {showBars ? <SideBar /> : null}
                {showBars ? <StatusBar meters={meters} /> : null}
                {showRoutingBar ? <RouteEditingBar /> : null}
                <Scene
                    width={dimension.width}
                    height={dimension.height}
                    options={options}
                    invisible={!isInitialized}/>
                {showLoader ? <Loader /> : null}
            </div>
        );
    }
}
