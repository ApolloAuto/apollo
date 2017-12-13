import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

class SideBarButton extends React.Component {
    render() {
        const { disabled, onClick, active, label, extraClasses } = this.props;
        return (
            <button onClick={onClick}
                    disabled={disabled}
                    className={classNames({
                            "button": true,
                            "active": active,
                        }, extraClasses)}>
                {label}
            </button>
        );
    }
}

export default class ButtonPanel extends React.Component {
    render() {
        const { enableHMIButtonsOnly,
                onTasks, showTasks,
                onModuleController, showModuleController,
                onMenu, showMenu,
                onRouteEditingBar, showRouteEditingBar,
                onPOI, showPOI } = this.props;

        return (
            <div>
                <SideBarButton label="Tasks"
                               disabled={false}
                               onClick={onTasks}
                               active={showTasks}/>
                <SideBarButton label="Module Controller"
                               disabled={false}
                               onClick={onModuleController}
                               active={showModuleController}/>
                <SideBarButton label="Layer Menu"
                               disabled={enableHMIButtonsOnly}
                               onClick={onMenu}
                               active={showMenu} />
                <SideBarButton label="Route Editing"
                               disabled={enableHMIButtonsOnly}
                               onClick={onRouteEditingBar}
                               active={showRouteEditingBar} />
                <SideBarButton label="Default Routing"
                               disabled={enableHMIButtonsOnly}
                               onClick={onPOI}
                               active={showPOI} />
            </div>
        );
    }
}
