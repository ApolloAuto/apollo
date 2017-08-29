import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

@observer
class SideBarButton extends React.Component {
    render() {
        const { onClick, active, label, extraClasses } = this.props;
        return (
            <button onClick={onClick}
                    className={classNames({
                            "button": true,
                            "active": active,
                        }, extraClasses)}>
                {label}
            </button>
        );
    }
}

@observer
export default class ButtonPanel extends React.Component {
    openHMI() {
        const server = window.location.origin;
        const link = document.createElement("a");
        link.href = server;
        window.open(
            `http://${link.hostname}:8887`, "_self");
    }

    render() {
        const { onConsole, showConsole,
                onMenu, showMenu,
                showRouteEditingBar } = this.props;

        return (
            <div>
                <SideBarButton label="HMI Setup" active={false}
                               onClick={this.openHMI.bind(this)}
                               extraClasses={["button-corner"]} />
                <div className="separator" />
                <SideBarButton label="Route Editing"
                               onClick={showRouteEditingBar}
                               active={false} />
                <div className="separator" />
                <SideBarButton label="Notifications"
                               onClick={onConsole}
                               active={showConsole} />
                <div className="separator" />
                <SideBarButton label="Layer Menu"
                               onClick={onMenu}
                               active={showMenu} />
            </div>
        );
    }
}
