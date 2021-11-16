import React from 'react';
import { inject, observer } from 'mobx-react';

import EditingTip from 'components/RouteEditingBar/EditingTip';

import removeAllIcon from 'assets/images/routing/remove_all.png';
import removeLastIcon from 'assets/images/routing/remove_last.png';
import sendRouteIcon from 'assets/images/routing/send_request.png';
import addPoiIcon from 'assets/images/routing/add_poi.png';
import inDefaultRoutingModeIcon from 'assets/images/routing/in_default_routing_mode.png';
import exitDefaultRoutingModeIcon from 'assets/images/routing/exit_default_routing_mode.png';

class RouteEditingButton extends React.Component {
  render() {
    const { label, icon, onClick, disabled } = this.props;

    return (
            <button onClick={onClick} className="button" disabled={disabled}>
                <img src={icon} />
                <span>{label}</span>
            </button>
    );
  }
}

@inject('store') @observer
export default class RouteEditingMenu extends React.Component {
  render() {
    const { routeEditingManager, options } = this.props.store;

    return (
            <div className="route-editing-bar">
                <div className="editing-panel">
                    <RouteEditingButton
                        label="Add Point of Interest"
                        icon={addPoiIcon}
                        onClick={() => {
                          this.props.store.handleOptionToggle('showPOI');
                        }}
                    />
                    <RouteEditingButton
                        label="Remove Last Point"
                        icon={removeLastIcon}
                        onClick={() => {
                          routeEditingManager.removeLastRoutingPoint();
                        }}
                    />
                    <RouteEditingButton
                        label="Remove All Points"
                        icon={removeAllIcon}
                        onClick={() => {
                          routeEditingManager.removeAllRoutingPoints();
                        }}
                    />
                    <RouteEditingButton
                        label="Send Routing Request"
                        icon={sendRouteIcon}
                        disabled={routeEditingManager.inDefaultRoutingMode}
                        onClick={() => {
                          if (routeEditingManager.sendRoutingRequest(false)) {
                            options.showRouteEditingBar = false;
                          }
                        }}
                    />
                    <RouteEditingButton
                        label="Add Default Routing"
                        icon={routeEditingManager.inDefaultRoutingMode
                          ? exitDefaultRoutingModeIcon : inDefaultRoutingModeIcon}
                        onClick={() => {
                          if (routeEditingManager.inDefaultRoutingMode) {
                            options.showDefaultRoutingInput = true;
                          }
                          else {
                            routeEditingManager.removeAllRoutingPoints();
                          }
                          routeEditingManager.toggleDefaultRoutingMode();
                        }}
                    />
                    <EditingTip />
                </div>
            </div>
    );
  }
}
