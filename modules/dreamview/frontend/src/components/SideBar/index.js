import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import Menu from "components/SideBar/Menu";
import Console from "components/SideBar/Console";
import Notification from "components/SideBar/Notification";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { monitor, options, routeEditingManager } = this.props.store;

        return (
            <div className="sidebar">
                <ButtonPanel showMenu={options.showMenu}
                             onMenu={() => {
                                     options.toggleShowMenu();
                                 }}
                             showConsole={options.showConsole}
                             onConsole={() => {
                                     options.toggleShowConsole();
                                 }}
                            showRouteEditingBar={() => {
                                      routeEditingManager.enableRouteEditing();
                                 }}/>
                {options.showMenu ? <Menu options={options} /> : <div/>}
                {options.showConsole ? <Console monitor={monitor} /> :
                 <Notification monitor={monitor} />}
            </div>
        );
    }
}
