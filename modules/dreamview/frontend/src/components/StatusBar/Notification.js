import React from 'react';
import { observer } from 'mobx-react';
import classNames from 'classnames';

import warnIcon from 'assets/images/icons/warning.png';
import errorIcon from 'assets/images/icons/error.png';

@observer
export default class Notification extends React.Component {
  render() {
    const { monitor, showPlanningRSSInfo } = this.props;

    if (!monitor.hasActiveNotification) {
      return null;
    }

    if (monitor.items.length === 0) {
      return null;
    }

    const item = monitor.items[0];

    const levelClass = (item.logLevel === 'ERROR'
                            || item.logLevel === 'FATAL')
      ? 'alert' : 'warn';
    const icon = levelClass === 'alert' ? errorIcon : warnIcon;

    return (
            <div
                className={`notification-${levelClass}`}
                style={{ right: showPlanningRSSInfo ? '500px' : '260px' }}
            >
                <img src={icon} className="icon" />
                <span className={classNames('text', levelClass)}>
                    {item.msg}
                </span>
            </div>
    );
  }
}
