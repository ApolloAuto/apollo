import React, { Component } from 'react';
import { inject, observer } from 'mobx-react';

const themeColor = ['white-theme', 'red-theme', 'orange-theme'];
@inject('store') @observer
export default class ManualCompetition extends Component {
  constructor(props) {
    super(props);
    this.state = {
      teamNumber: '',
      confirmNumber: false,
    };

    this.renderManualCompetition = this.renderManualCompetition.bind(this);
    this.handleInput = (event) => {
      this.setState({ teamNumber: event.target.value });
    };
    this.determinTeamNumber = this.determinTeamNumber.bind(this);
    this.resetTeamNumber = this.resetTeamNumber.bind(this);
  }

  determinTeamNumber() {
    this.props.store.hmi.setTeamNumber(this.state.teamNumber);
    this.setState({ confirmNumber: true });
  }

  resetTeamNumber() {
    this.props.store.hmi.clearTeamNumber();
    this.setState({
      confirmNumber: false,
      teamNumber: '',
    });
  }

  renderManualCompetition() {
    const {
      velometerSpeed, behavior, isOverspeed, overspeedCount, outCount, moduleStatus,
    } = this.props.store.hmi;
    const showInfo = this.state.teamNumber && this.state.confirmNumber;
    const speedColor = `speed-section ${_.get(themeColor, isOverspeed, 0)}`;
    return (
      <div className="monitor-content">
        <div className={speedColor}>
          {showInfo ? velometerSpeed : ''}
          {showInfo && (<span className="speed-unit">m/s</span>)}
        </div>
        <div className="monitor-row section">
          <label className="one"><span className="label-txt">当前车辆所在位置</span></label>
          <span className="two">{showInfo ? behavior : ''}</span>
        </div>
        <div className="monitor-row section">
          <label className="one"><span className="label-txt">超出赛道次数</span></label>
          <span className="two">{showInfo ? outCount : ''}</span>
        </div>
        <div className="monitor-row foul-section">
          <label className="one">
            <span className="label-txt label-txt-foul">犯规次数</span>
          </label>
          <span className="two">{showInfo ? overspeedCount : ''}</span>
        </div>
        <div className="monitor-row number-section">
          <label className="number-label"><span className="label-txt">请输入参赛队伍编号</span></label>
          <input
            className="number-input"
            disabled={this.state.confirmNumber}
            value={this.state.teamNumber}
            onChange={this.handleInput}
          ></input>
          <button
            className="number-btn"
            disabled={this.state.confirmNumber}
            onClick={this.determinTeamNumber}
          >
            确定
          </button>
          <button
            className="number-btn"
            disabled={moduleStatus.get('Start Competition') || !this.state.confirmNumber}
            onClick={this.resetTeamNumber}
          >
            重置
          </button>
        </div>
      </div>
    );
  }

  render() {
    return (
            <div className="monitor manual-competition">
                {this.renderManualCompetition()}
            </div>
    );
  }
}
