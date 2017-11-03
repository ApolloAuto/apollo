import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import POI from "components/SideBar/POI";
import Menu from "components/SideBar/Menu";
import Console from "components/SideBar/Console";
import Notification from "components/SideBar/Notification";
import WS from "store/websocket";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { monitor, options, routeEditingManager, video } = this.props.store;

        return (
            <div className="side-bar">
                <ButtonPanel resetBackend={() => {
                                     WS.resetBackend();
                                 }}
                             dumpMessages={() => {
                                     WS.dumpMessages();
                                 }}
                             onPOI={() => {
                                 options.toggleShowPOI();
                             }}
                             showPOI={options.showPOI}
                             showRouteEditingBar={() => {
                                     options.showPOI = false;
                                     routeEditingManager.enableRouteEditing();
                                 }}
                             onVideo={(event) => {
                                     video.setVideo(event.target.files[0]);
                                 }}
                             onPNCMonitor={() => {
                                     this.props.store.setPNCMonitor();
                                 }}
                             showPNCMonitor={options.showPNCMonitor}
                             onConsole={() => {
                                     options.toggleShowConsole();
                                 }}
                             showConsole={options.showConsole}
                             onMenu={() => {
                                     options.toggleShowMenu();
                                 }}
                             showMenu={options.showMenu} />
                {options.showPOI ? <POI routeEditingManager={routeEditingManager}
                    options={options} /> : <div/>}
                {options.showMenu ? <Menu options={options} /> : <div/>}
                {options.showConsole ? <Console monitor={monitor} /> :
                 <Notification monitor={monitor} />}
            </div>
        );
    }
}
