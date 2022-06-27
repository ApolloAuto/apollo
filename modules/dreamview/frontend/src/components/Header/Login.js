import React from 'react';
import { inject, observer } from 'mobx-react';
import { ACCOUNT_WS } from 'store/websocket';

const DREAMVIEW_KEY = 'dreamview';
const LOGIN_ENTRYPOINT = 'https://studio.apollo.auto/user/login';

@inject('store') @observer
export default class Login extends React.Component {

  renderLogout() {
    const { account } = this.props.store;
    return (
      <div className="header-nav-logout">
        <p style={{cursor: 'pointer'}}>
          <span onClick={() => ACCOUNT_WS.refreshAccountInfo()}>{account.username}</span>
          <span onClick={() => ACCOUNT_WS.requestLogoutAccount()} className="header-nav-logout-sublist">
            logout
          </span>
        </p>
      </div>
    );
  }

  renderLogin() {
    return (
      <button className="header-item header-button"
        onClick={() => {
          const { protocol, host } = window.location;
          const callbackUrl = `${protocol}//${host}/login_callback`;
          const loginUrl = `${LOGIN_ENTRYPOINT}?callbackUrl=${encodeURIComponent(callbackUrl)}`;
          const windowFeatures = [
            'directories=no',
            'titlebar=no',
            'toolbar=no',
            'location=no',
            'status=no',
            'menubar=no',
            'scrollbars=no',
            'resizable=no',
          ];
          window.open(loginUrl, DREAMVIEW_KEY, windowFeatures.join(','));
        }}
      >
        Login
      </button>
    );
  }

  render() {
    const { account } = this.props.store;
    return (
      <React.Fragment>
        { account.isLogged ? this.renderLogout() : this.renderLogin() }
      </React.Fragment>
    );
  }
}
