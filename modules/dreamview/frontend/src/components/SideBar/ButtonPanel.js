import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

import TasksIcon from "assets/images/sidebar/tasks.png";
import ModuleControllerIcon from "assets/images/sidebar/module_controller.png";
import LayerMenuIcon from "assets/images/sidebar/layer_menu.png";
import RouteEditingIcon from "assets/images/sidebar/route_editing.png";
import DataRecorderIcon from "assets/images/sidebar/data_recorder.png";

class SideBarButton extends React.PureComponent {
    render() {
        const { disabled, onClick, active, label, extraClasses, iconSrc } = this.props;
        return (
            <button onClick={onClick}
                    disabled={disabled}
                    className={classNames({
                            "button": true,
                            "button-active": active,
                        }, extraClasses)}>
                <img src={iconSrc} className="icon" />
                <div className="label">{label}</div>
            </button>
        );
    }
}

export default class ButtonPanel extends React.PureComponent {
    render() {
        const { enableHMIButtonsOnly, inNavigationMode,
                onTasks, showTasks,
                onModuleController, showModuleController,
                onMenu, showMenu,
                onRouteEditingBar, showRouteEditingBar,
                onDataRecorder, showDataRecorder } = this.props;

        return (
            <div className="main-panel">
                <SideBarButton label="Tasks"
                               disabled={false}
                               iconSrc={TasksIcon}
                               onClick={onTasks}
                               active={showTasks}/>
                <SideBarButton label="Module Controller"
                               disabled={false}
                               iconSrc={ModuleControllerIcon}
                               onClick={onModuleController}
                               active={showModuleController}/>
                <SideBarButton label="Layer Menu"
                               disabled={enableHMIButtonsOnly}
                               iconSrc={LayerMenuIcon}
                               onClick={onMenu}
                               active={showMenu} />
                <SideBarButton label="Route Editing"
                               disabled={enableHMIButtonsOnly || inNavigationMode}
                               iconSrc={RouteEditingIcon}
                               onClick={onRouteEditingBar}
                               active={showRouteEditingBar} />
                <SideBarButton label="Data Recorder"
                               disabled={enableHMIButtonsOnly}
                               iconSrc={DataRecorderIcon}
                               onClick={onDataRecorder}
                               active={showDataRecorder} />
            </div>
        );
    }
}
