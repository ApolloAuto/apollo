import React from "react";
import { inject, observer } from "mobx-react";
import ReactTooltip from "react-tooltip";
import _ from "lodash";

import SideBarButton from "components/SideBar/SideBarButton";
import HOTKEYS_CONFIG from "store/config/hotkeys.yml";
import WS from "store/websocket";

import TasksIcon from "assets/images/sidebar/tasks.png";
import ModuleControllerIcon from "assets/images/sidebar/module_controller.png";
import LayerMenuIcon from "assets/images/sidebar/layer_menu.png";
import RouteEditingIcon from "assets/images/sidebar/route_editing.png";
import DataRecorderIcon from "assets/images/sidebar/data_recorder.png";

const sidebarIconMapping = {
    showTasks: TasksIcon,
    showModuleController: ModuleControllerIcon,
    showMenu: LayerMenuIcon,
    showRouteEditingBar: RouteEditingIcon,
    showDataRecorder: DataRecorderIcon,
};

const sidebarLabelMapping = {
    showTasks: "Tasks",
    showModuleController: "Module Controller",
    showMenu: "Layer Menu",
    showRouteEditingBar: "Route Editing",
    showDataRecorder: "Data Recorder",
    showPOI: "Default Routing",
};

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly, hmi } = this.props.store;

        const hotkeys = _.invert(HOTKEYS_CONFIG);
        const settings = {};
        const optionNames = [...options.mainSideBarOptions, ...options.secondarySideBarOptions];
        optionNames.forEach(optionName => {
            settings[optionName] = {
                label: sidebarLabelMapping[optionName],
                active: options[optionName],
                onClick: () => {
                    this.props.store.handleOptionToggle(optionName);
                },
                disabled: options.isSideBarButtonDisabled(
                    optionName,
                    enableHMIButtonsOnly,
                    hmi.inNavigationMode
                ),
                hotkey: hotkeys[optionName],
                iconSrc: sidebarIconMapping[optionName],
            };
        });

        return (
            <div className="side-bar">
                <div className="main-panel">
                    <SideBarButton type="main" {...settings.showTasks} />
                    <SideBarButton type="main" {...settings.showModuleController} />
                    <SideBarButton type="main" {...settings.showMenu} />
                    <SideBarButton type="main" {...settings.showRouteEditingBar} />
                    <SideBarButton type="main" {...settings.showDataRecorder} />
                </div>
                <div className="sub-button-panel">
                    <SideBarButton
                        type="sub" {...settings.showPOI}
                        active={!options.showRouteEditingBar && options.showPOI} />
                </div>
                <ReactTooltip id="sidebar-button" place="right" delayShow={500} />
            </div>
        );
    }
}
