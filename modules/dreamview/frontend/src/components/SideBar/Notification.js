import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

import Image from "components/common/Image";
import warnIcon from "assets/images/icons/warning.png";
import errorIcon from "assets/images/icons/error.png";

@observer
export default class Notification extends React.Component {
    render() {
        const { monitor } = this.props;

        if (!monitor.hasActiveNotification) {
            return null;
        }

        if (monitor.items.length === 0) {
            return null;
        }

        const item = monitor.items[0];
        const levelClass = (item.logLevel === "ERROR" ||
                            item.logLevel === "FATAL") ?
                           "alert" : "warn";
        const icon = levelClass === "alert" ? errorIcon : warnIcon;

        return (
            <div className={`notification-${levelClass}`}>
                <Image image={icon} className="icon" />
                <span className={classNames("text", levelClass)}>
                    {item.msg}
                </span>
            </div>
        );
    }
}
