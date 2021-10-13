import React from 'react';
import _ from 'lodash';

import RadioItem from 'components/common/RadioItem';

export default class DefaultRoutingInput extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      routingType: 'defaultRouting',
    };

    this.saveDefaultRouting = this.saveDefaultRouting.bind(this);
    this.removeDefaultRouting = this.removeDefaultRouting.bind(this);
    this.changeRoutingType = this.changeRoutingType.bind(this);
  }

  saveDefaultRouting() {
    const { routeEditingManager, options } = this.props;
    const routingName = document.getElementsByClassName('name-input')[0].value;
    if (_.isEmpty(routingName)) {
      alert('The default routing name is necessary');
    }
    else {
      routeEditingManager.addDefaultRouting(routingName, this.state.routingType);
      options.showDefaultRoutingInput = false;
      routeEditingManager.removeAllRoutingPoints();
    }
  }

  removeDefaultRouting() {
    const { routeEditingManager, options } = this.props;
    routeEditingManager.removeAllRoutingPoints();
    options.showDefaultRoutingInput = false;
  }

  changeRoutingType(value) {
    this.setState({ routingType: value });
  }

  render() {
    return (
      <div className="default-routing-input">
        <div>
          <label className="name-label">Routing Name:</label>
          <input className="name-input"></input>
        </div>
        <div>
          <label className="name-label">Routing Type:</label>
          <RadioItem
            key="defaultRouting"
            extraClasses={['default-routing-input-radio']}
            id="routingType"
            onClick={() => { this.changeRoutingType('defaultRouting'); }}
            checked={this.state.routingType === 'defaultRouting'}
            title="Default Routing"
          />
          <RadioItem
            key="parkGoRouting"
            id="routingType"
            onClick={() => { this.changeRoutingType('parkGoRouting'); }}
            title="Park Go Routing"
            checked={this.state.routingType === 'parkGoRouting'}
          />
        </div>
        <div className="default-routing-input-btn">
          <button className="input-button submit-button" onClick={this.saveDefaultRouting}>Save</button>
          <button className="input-button" onClick={this.removeDefaultRouting}>Cancel</button>
        </div>
      </div>
    );
  }
}
