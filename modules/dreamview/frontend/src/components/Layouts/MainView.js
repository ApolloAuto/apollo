import React from "react";
import { inject, observer } from "mobx-react";

import Navigation from "components/Navigation";
import DashCamPlayer from "components/DashCamPlayer";
import RouteEditingBar from "components/RouteEditingBar";
import StatusBar from "components/StatusBar";
import Scene from "components/Scene";
import Loader from "components/common/Loader";
import PlaybackControls from "components/PlaybackControls";

@inject("store") @observer
class SceneView extends React.Component {
    render() {
        const { sceneDimension, meters, monitor, options, trafficSignal, video } = this.props.store;

        return (
            <div className="main-view" style={{height: sceneDimension.height}}>
                <Scene  width={sceneDimension.width}
                        height={sceneDimension.height}
                        options={options}
                        invisible={false}/>
                {options.showRouteEditingBar
                    ? <RouteEditingBar />
                    : <StatusBar meters={meters}
                                 trafficSignal={trafficSignal}
                                 showNotification={!options.showConsole}
                                 monitor={monitor}/>}
                {video.showVideo && <DashCamPlayer />}
                {OFFLINE_PLAYBACK && <PlaybackControls />}
            </div>
        );
    }
}

@inject("store") @observer
export default class MainView extends React.Component {
    render() {
        const { isInitialized, sceneDimension, hmi } = this.props.store;

        if (hmi.showNavigationMap) {
            return <Navigation height={sceneDimension.height}/>;
        } else if (!isInitialized) {
            return <Loader height={sceneDimension.height}/>;
        } else {
            return <SceneView />;
        }
    }
}

