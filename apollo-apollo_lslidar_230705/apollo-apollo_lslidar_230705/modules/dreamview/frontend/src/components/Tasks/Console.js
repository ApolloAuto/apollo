import React from 'react';
import { inject, observer } from 'mobx-react';
import classNames from 'classnames';

import warnIcon from 'assets/images/icons/warning.png';
import errorIcon from 'assets/images/icons/error.png';
import successIcon from 'assets/images/icons/success.png';
import notificationIcon from 'assets/images/icons/notification.png';
import { timestampMsToTimeString } from 'utils/misc';

const LEVEL_CLASS = Object.freeze({
  ERROR: 'alert',
  FATAL: 'alert',
  FAIL: 'alert',
  SUCCESS: 'success',
  UNKNOWN: 'notification',
});

const LEVEL_ICON = Object.freeze({
  ERROR: errorIcon,
  FATAL: errorIcon,
  FAIL: errorIcon,
  SUCCESS: successIcon,
  UNKNOWN: notificationIcon,
});
//Todo:level statement style

@observer
export class MonitorItem extends React.Component {
  render() {
    const { level, text, time } = this.props;

    const levelClass = LEVEL_CLASS[level] ? LEVEL_CLASS[level] : 'warn';
    const icon = LEVEL_ICON[level] ? LEVEL_ICON[level] : warnIcon;

    return (
            <li className="monitor-item">
                <img src={icon} className="icon" />
                <span className={classNames('text', levelClass)}>
                    {text}
                </span>
                <span className={classNames('time', levelClass)}>{time}</span>
            </li>
    );
  }
}

@inject('store') @observer
export default class Console extends React.Component {
  render() {
    const { monitor } = this.props.store;

    return (
            <div className="card" style={{ maxWidth: '50%' }}>
                <div className="card-header"><span>Console</span></div>
                <div className="card-content-column">
                    <ul className="console">
                        {monitor.items.map((item, index) => (
                            <MonitorItem
                                key={index}
                                text={item.msg}
                                level={item.logLevel}
                                time={timestampMsToTimeString(item.timestampMs)}
                            />
                        ))}
                    </ul>
                </div>
            </div>
    );
  }
}
