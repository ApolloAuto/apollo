import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import WS from "store/websocket";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly } = this.props.store;

        return (
            <div className="side-bar">
                <ButtonPanel enableHMIButtonsOnly={enableHMIButtonsOnly}
                             onTasks={() => {
                                this.props.store.handleSideBarClick('showTasks');
                             }}
                             showTasks={options.showTasks}
                             onModuleController={() => {
                                this.props.store.handleSideBarClick('showModuleController');
                             }}
                             showModuleController={options.showModuleController}
                             onMenu={() => {
                                    this.props.store.handleSideBarClick('showMenu');
                                 }}
                             showMenu={options.showMenu}
                             onRouteEditingBar={() => {
                                    this.props.store.handleSideBarClick('showRouteEditingBar');
                                 }}
                             showRouteEditingBar={options.showRouteEditingBar}
                             onPOI={() => {
                                 this.props.store.handleSideBarClick('showPOI');
                             }}
                             showPOI={options.showPOI} />
            </div>
        );
    }
}
