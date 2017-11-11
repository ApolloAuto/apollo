import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import WS from "store/websocket";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { isInitialized, options, routeEditingManager, video } = this.props.store;

        return (
            <div className="side-bar">
                <ButtonPanel initialized={isInitialized}
                             onQuickStarter={() => {
                                this.props.store.handleSideBarClick('showQuickStarter');
                             }}
                             showQuickStarter={options.showQuickStarter}
                             onModuleController={() => {
                                this.props.store.handleSideBarClick('showModuleController');
                             }}
                             showModuleController={options.showModuleController}
                             resetBackend={() => {
                                     WS.resetBackend();
                                 }}
                             dumpMessages={() => {
                                     WS.dumpMessages();
                                 }}
                             onDefaultRouting={() => {
                                 if (routeEditingManager.addDefaultEndPoint()) {
                                     if (!options.showRouteEditingBar) {
                                        routeEditingManager.sendRoutingRequest();
                                    }
                                 }
                             }}
                             onRouteEditingBar={() => {
                                    this.props.store.handleSideBarClick('showRouteEditingBar');
                                 }}
                             showRouteEditingBar={options.showRouteEditingBar}
                             onVideo={(event) => {
                                     video.setVideo(event.target.files[0]);
                                 }}
                             onPNCMonitor={() => {
                                     this.props.store.handleSideBarClick('showPNCMonitor');
                                 }}
                             showPNCMonitor={options.showPNCMonitor}
                             onConsole={() => {
                                     this.props.store.handleSideBarClick('showConsole');
                                 }}
                             showConsole={options.showConsole}
                             onMenu={() => {
                                    this.props.store.handleSideBarClick('showMenu');
                                 }}
                             showMenu={options.showMenu} />
            </div>
        );
    }
}
