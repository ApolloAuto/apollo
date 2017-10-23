import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import Menu from "components/SideBar/Menu";
import Console from "components/SideBar/Console";
import Notification from "components/SideBar/Notification";
import WS from "store/websocket";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { monitor, options, routeEditingManager, video } = this.props.store;

        return (
            <div className="sidebar">
                <ButtonPanel resetBackend={() => {
                                     WS.resetBackend();
                                 }}
                             dumpMessages={() => {
                                     WS.dumpMessages();
                                 }}
                             sendDefaultRoutingRequest={() => {
                                     routeEditingManager.sendRoutingRequest(true);
                                 }}
                             showRouteEditingBar={() => {
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
                {options.showMenu ? <Menu options={options} /> : <div/>}
                {options.showConsole ? <Console monitor={monitor} /> :
                 <Notification monitor={monitor} />}
            </div>
        );
    }
}
