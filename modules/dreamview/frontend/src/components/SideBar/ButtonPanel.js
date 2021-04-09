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
        const { settings } = this.props;

        return (
            <div className="main-panel">
                <SideBarButton label="Tasks"
                               disabled={settings.showTasks.disabled}
                               iconSrc={TasksIcon}
                               onClick={settings.showTasks.onClick}
                               active={settings.showTasks.active} />
                <SideBarButton label="Module Controller"
                               disabled={settings.showModuleController.disabled}
                               iconSrc={ModuleControllerIcon}
                               onClick={settings.showModuleController.onClick}
                               active={settings.showModuleController.active} />
                <SideBarButton label="Layer Menu"
                               disabled={settings.showMenu.disabled}
                               iconSrc={LayerMenuIcon}
                               onClick={settings.showMenu.onClick}
                               active={settings.showMenu.active} />
                <SideBarButton label="Route Editing"
                               disabled={settings.showRouteEditingBar.disabled}
                               iconSrc={RouteEditingIcon}
                               onClick={settings.showRouteEditingBar.onClick}
                               active={settings.showRouteEditingBar.active} />
                <SideBarButton label="Data Recorder"
                               disabled={settings.showDataRecorder.disabled}
                               iconSrc={DataRecorderIcon}
                               onClick={settings.showDataRecorder.onClick}
                               active={settings.showDataRecorder.active} />
            </div>
        );
    }
}
