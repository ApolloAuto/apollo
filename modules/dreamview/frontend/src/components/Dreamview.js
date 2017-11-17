import React from "react";
import { inject, observer } from "mobx-react";
import SplitPane from 'react-split-pane';

import DashCamPlayer from "components/DashCamPlayer";
import ModuleController from "components/ModuleController";
import Navigation from "components/Navigation";
import PNCMonitor from "components/PNCMonitor";
import RouteEditingBar from "components/RouteEditingBar";
import QuickStarter from "components/QuickStarter";
import Header from "components/Header";
import Loader from "components/common/Loader";
import SideBar from "components/SideBar";
import Console from "components/SideBar/Console";
import POI from "components/SideBar/POI";
import Menu from "components/SideBar/Menu";
import StatusBar from "components/StatusBar";
import Scene from "components/Scene";
import WS from "store/websocket";

@inject("store") @observer
class MainView extends React.Component {
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
            </div>
        );
    }
}

@inject("store") @observer
class Tools extends React.Component {
    render() {
        const { monitor, options, routeEditingManager } = this.props.store;

        return (
            <div className="tools">
                {options.showModuleController && <ModuleController />}
                {options.showQuickStarter && <QuickStarter />}
                {options.showMenu && <Menu options={options} /> }
                {options.showPOI && <POI routeEditingManager={routeEditingManager}
                                         options={options}/>}
                {options.showConsole && <Console monitor={monitor} />}
            </div>
        );
    }
}

@inject("store") @observer
export default class Dreamview extends React.Component {
    constructor(props) {
        super(props);
        this.handleDrag = this.handleDrag.bind(this);
    }

    handleDrag(masterViewWidth) {
        const { options } = this.props.store;
        if (options.showPNCMonitor) {
            this.props.store.updateWidthInPercentage(
                Math.min(1.00, masterViewWidth / window.innerWidth));
        }
    }

    componentWillMount() {
        this.props.store.updateDimension();
    }

    componentDidMount() {
        WS.initialize();
        window.addEventListener("resize", () => {
            this.props.store.updateDimension();
        });
    }

    render() {
        const { isInitialized, dimension, sceneDimension, options, hmi } = this.props.store;

        let mainView = null;
        if (hmi.showNavigationMap) {
            mainView = <Navigation height={sceneDimension.height}/>;
        } else if (!isInitialized) {
            mainView = <Loader height={sceneDimension.height}/>;
        } else {
            mainView = <MainView />;
        }
        return (
            <div>
                <Header />
                <div className="pane-container">
                    <SplitPane split="vertical"
                           size={dimension.width}
                           onChange={this.handleDrag}
                           allowResize={options.showPNCMonitor}>
                        <div className="left-pane">
                            <SideBar />
                            <div className="dreamview-body">
                                {mainView}
                                <Tools />
                            </div>
                        </div>
                        <div className="right-pane">
                            {options.showPNCMonitor && <PNCMonitor />}
                        </div>
                    </SplitPane>
                </div>
            </div>
        );
    }
}