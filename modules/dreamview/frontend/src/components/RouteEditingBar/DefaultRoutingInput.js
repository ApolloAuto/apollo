import React from 'react';
import _ from 'lodash';

export default class DefaultRoutingInput extends React.Component {
  constructor(props) {
    super(props);

    this.saveDefaultRouting = this.saveDefaultRouting.bind(this);
    this.removeDefaultRouting = this.removeDefaultRouting.bind(this);
  }

  saveDefaultRouting() {
    const { routeEditingManager, options } = this.props;
    const routingName = document.getElementsByClassName('name-input')[0].value;
    if (_.isEmpty(routingName)) {
      alert('The default routing name is necessary');
    }
    else {
      routeEditingManager.addDefaultRouting(routingName);
      options.showDefaultRoutingInput = false;
      routeEditingManager.removeAllRoutingPoints();
    }
  }

  removeDefaultRouting() {
    const { routeEditingManager, options } = this.props;
    routeEditingManager.removeAllRoutingPoints();
    options.showDefaultRoutingInput = false;
  }

  render() {
    return (
      <div className="default-routing-input">
        <div>
          <label className="name-label">Default Routing Name:</label>
          <input className="name-input"></input>
        </div>
        <div className="default-routing-input-btn">
          <button className="input-button submit-button" onClick={this.saveDefaultRouting}>Save</button>
          <button className="input-button" onClick={this.removeDefaultRouting}>Cancel</button>
        </div>
      </div>
    );
  }
}
