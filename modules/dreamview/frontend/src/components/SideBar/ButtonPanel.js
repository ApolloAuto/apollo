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
        const { onConsole, showConsole } = this.props;

        return (
            <div>
                <SideBarButton label="HMI Setup" active={false}
                               onClick={this.openHMI.bind(this)} />
                <div className="separator" />
                <SideBarButton label="Notifications"
                               onClick={onConsole}
                               active={showConsole}
                               extraClasses={["button-corner"]} />
            </div>
        );
    }
}
