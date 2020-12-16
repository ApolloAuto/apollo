import React from 'react';
import _ from 'lodash';

export default class CycleNumberInput extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      cycleNumber: 1,
    };
    this.sendCycleDefaultRouting = this.sendCycleDefaultRouting.bind(this);
    this.cancelSendDefaultRouting = this.cancelSendDefaultRouting.bind(this);
    this.handleInput = (event) => {
      this.setState({ cycleNumber: event.target.value });
    };
  }

  sendCycleDefaultRouting() {
    const { routeEditingManager, options } = this.props;
    const cycleNumber = parseInt(this.state.cycleNumber, 10);
    if (isNaN(cycleNumber) || cycleNumber < 1) {
      alert('please input a valid cycle number');
    }
    else {
      routeEditingManager.sendCycleRoutingRequest(routeEditingManager.currentDefaultRouting,
        cycleNumber);
    }
    options.showCycleNumberInput = false;
  }

  cancelSendDefaultRouting() {
    const { options } = this.props;
    options.showCycleNumberInput = false;
  }

  render() {
    return (
      <React.Fragment>
        <div className="default-routing-input">
          <div>
            <label className="name-label">Cycle Number:</label>
            <input
              className="name-input"
              value={this.state.cycleNumber}
              onChange={this.handleInput}
              type="number"
            ></input>
          </div>
          <div className="default-routing-input-btn">
            <button className="input-button submit-button" onClick={this.sendCycleDefaultRouting}>Send</button>
            <button className="input-button" onClick={this.cancelSendDefaultRouting}>Cancel</button>
          </div>
        </div>
      </React.Fragment>
    );
  }
}
