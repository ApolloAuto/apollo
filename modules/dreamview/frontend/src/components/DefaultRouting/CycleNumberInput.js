import React from 'react';
import _ from 'lodash';

import CheckboxItem from 'components/common/CheckboxItem';

export default class CycleNumberInput extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      cycleNumber: 1,
      isCycling: false,
    };
    this.sendCycleDefaultRouting = this.sendCycleDefaultRouting.bind(this);
    this.cancelSendDefaultRouting = this.cancelSendDefaultRouting.bind(this);
    this.toggleCycle = this.toggleCycle.bind(this);
    this.handleInput = (event) => {
      this.setState({ cycleNumber: event.target.value });
    };
  }

  toggleCycle() {
    this.setState((prevState) => {
      return { isCycling: !prevState.isCycling };
    });
  }

  sendCycleDefaultRouting() {
    const { routeEditingManager, options } = this.props;
    if (this.state.isCycling) {
      const cycleNumber = parseInt(this.state.cycleNumber, 10);
      if (isNaN(cycleNumber) || cycleNumber < 1) {
        alert('please input a valid cycle number');
      }
      else if (!routeEditingManager.checkCycleRoutingAvailable()) {
        alert(`Please set the default routing reasonably,the distance from the car position
          to the end point should exceed ${routeEditingManager.defaultRoutingDistanceThreshold},
          otherwise it will not be able to form a closed loop.`);
      }
      else {
        routeEditingManager.sendCycleRoutingRequest(cycleNumber);
      }
    }
    else {
      routeEditingManager.sendRoutingRequest(false, routeEditingManager.currentDefaultRouting);
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
            <div>
              <label className="name-label">Start Cycling</label>
              <CheckboxItem
                extraClasses="start-cycle-checkbox"
                id="isReportableData"
                isChecked={this.state.isCycling}
                disabled={false}
                onClick={this.toggleCycle}
              />
            </div>
            {this.state.isCycling &&
              <div>
                <label className="name-label">Cycle Number:</label>
                <input
                  className="name-input"
                  value={this.state.cycleNumber}
                  onChange={this.handleInput}
                  type="number"
                ></input>
              </div>}
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
