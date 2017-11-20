import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

import Image from "components/common/Image";
import warnIcon from "assets/images/icons/warning.png";
import errorIcon from "assets/images/icons/error.png";

@observer
class MonitorItem extends React.Component {
    render() {
        const { level, text } = this.props;

        const levelClass = (level === "ERROR" || level === "FATAL") ?
                           "alert" : "warn";
        const icon = levelClass === "alert" ? errorIcon : warnIcon;

        return (
            <li className="monitor-item">
                <Image image={icon} className="icon" />
                <span className={classNames("text", levelClass)}>
                    {text}
                </span>
            </li>
        );
    }
}

@observer
export default class Console extends React.Component {
    render() {
        const { monitor } = this.props;

        return (
            <ul className="console card">
                {monitor.items.map((item, index) => (
                     <MonitorItem key={index} text={item.msg}
                                  level={item.logLevel} />
                 ))}
            </ul>
        );
    }
}
