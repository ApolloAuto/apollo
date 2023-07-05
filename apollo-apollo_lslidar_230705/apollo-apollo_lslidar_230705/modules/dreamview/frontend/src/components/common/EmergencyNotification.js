import React from 'react';

import errorIcon from 'assets/images/icons/error.png';

export default class EmergencyNotification extends React.PureComponent {
  render() {
    const { msg } = this.props;
    if (!msg) {
      return null;
    }

    return (
      <div className="notification-alert-center">
        <img src={errorIcon} className="icon" />
        <span className="text alert">{msg}</span>
      </div>
    );
  }
}
