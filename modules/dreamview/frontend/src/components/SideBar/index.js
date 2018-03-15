import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import SubButtonPanel from "components/SideBar/SubButtonPanel";
import WS from "store/websocket";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly, hmi } = this.props.store;

        return (
            <div className="side-bar">
                <ButtonPanel enableHMIButtonsOnly={enableHMIButtonsOnly}
                             inNavigationMode={hmi.inNavigationMode}
                             onTasks={() => {
                                this.props.store.handleOptionToggle('showTasks');
                             }}
                             showTasks={options.showTasks}
                             onModuleController={() => {
                                this.props.store.handleOptionToggle('showModuleController');
                             }}
                             showModuleController={options.showModuleController}
                             onMenu={() => {
                                    this.props.store.handleOptionToggle('showMenu');
                                 }}
                             showMenu={options.showMenu}
                             onRouteEditingBar={() => {
                                    this.props.store.handleOptionToggle('showRouteEditingBar');
                                 }}
                             showRouteEditingBar={options.showRouteEditingBar}
                             onDataRecorder={() => {
                                    this.props.store.handleOptionToggle('showDataRecorder');
                                 }}
                             showDataRecorder={options.showDataRecorder} />
                <SubButtonPanel panelLabel="Voice Command"
                                enablePanel={true}
                                onPanel={() => {
                                    this.props.store.handleOptionToggle('enableVoiceCommand');
                                }}
                                showPanel={options.enableVoiceCommand} />
                <SubButtonPanel panelLabel="Default Routing"
                                enablePanel={
                                    !enableHMIButtonsOnly && !options.showRouteEditingBar
                                }
                                onPanel={() => {
                                    this.props.store.handleOptionToggle('showPOI');
                                }}
                                showPanel={!options.showRouteEditingBar && options.showPOI} />
            </div>
        );
    }
}
