import React from 'react';
import _ from 'lodash';

import CheckboxItem from 'components/common/CheckboxItem';

export default class ParkTimeInput extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      parkTime: 1,
    };
    this.sendParkGoRouting = this.sendParkGoRouting.bind(this);
    this.cancelSendParkGoRouting = this.cancelSendParkGoRouting.bind(this);
    this.handleInput = (event) => {
      this.setState({ parkTime: event.target.value });
    };
  }

  sendParkGoRouting() {
    const { routeEditingManager, options } = this.props;
    const parkTime = parseInt(this.state.parkTime, 10);
    if (isNaN(parkTime) || parkTime < 1) {
      alert('please input a valid park time');
    } else {
      routeEditingManager.sendParkGoRoutingRequest(parkTime);
    }
    options.showParkTimeInput = false;
  }

  cancelSendParkGoRouting() {
    const { options } = this.props;
    options.showParkTimeInput = false;
  }

  render() {
    return (
      <React.Fragment>
        <div className="default-routing-input">
          <div>
                <label className="name-label">Park Time:</label>
                <input
                  className="name-input"
                  value={this.state.parkTime}
                  onChange={this.handleInput}
                  type="number"
                ></input>s
          </div>
          <div className="default-routing-input-btn">
            <button className="input-button submit-button" onClick={this.sendParkGoRouting}>Send</button>
            <button className="input-button" onClick={this.cancelSendParkGoRouting}>Cancel</button>
          </div>
        </div>
      </React.Fragment>
    );
  }
}
