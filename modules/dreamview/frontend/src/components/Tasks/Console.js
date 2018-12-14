import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";

import warnIcon from "assets/images/icons/warning.png";
import errorIcon from "assets/images/icons/error.png";
import { timestampMsToTimeString } from "utils/misc";

@observer
export class MonitorItem extends React.Component {
    render() {
        const { level, text, time } = this.props;

        const levelClass = (level === "ERROR" || level === "FATAL") ?
                           "alert" : "warn";
        const icon = levelClass === "alert" ? errorIcon : warnIcon;

        return (
            <li className="monitor-item">
                <img src={icon} className="icon" />
                <span className={classNames("text", levelClass)}>
                    {text}
                </span>
                <span className={classNames("time", levelClass)}>{time}</span>
            </li>
        );
    }
}

@inject("store") @observer
export default class Console extends React.Component {
    render() {
        const { monitor } = this.props.store;

        return (
            <div className="card" style={{maxWidth: '50%'}}>
                <div className="card-header"><span>Console</span></div>
                <div className="card-content-column">
                    <ul className="console">
                        {monitor.items.map((item, index) => (
                            <MonitorItem key={index} text={item.msg}
                                level={item.logLevel}
                                time={timestampMsToTimeString(item.timestampMs)} />
                        ))}
                    </ul>
                </div>
            </div>
        );
    }
}
