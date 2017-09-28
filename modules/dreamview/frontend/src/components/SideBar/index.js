import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import Menu from "components/SideBar/Menu";
import Console from "components/SideBar/Console";
import Notification from "components/SideBar/Notification";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { monitor, options, routeEditingManager, video } = this.props.store;

        return (
            <div className="sidebar">
                <ButtonPanel showRouteEditingBar={() => {
                                     routeEditingManager.enableRouteEditing();
                                 }}
                             sendDefaultRoutingRequest={() => {
                                     routeEditingManager.sendRoutingRequest(true);
                                 }}
                             showMenu={options.showMenu}
                             onMenu={() => {
                                     options.toggleShowMenu();
                                 }}
                             showConsole={options.showConsole}
                             onConsole={() => {
                                     options.toggleShowConsole();
                                 }}
                             onVideo={(event) => {
                                     video.setVideo(event.target.files[0]);
                                 }}/>
                {options.showMenu ? <Menu options={options} /> : <div/>}
                {options.showConsole ? <Console monitor={monitor} /> :
                 <Notification monitor={monitor} />}
            </div>
        );
    }
}
