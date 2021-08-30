import React, { Component } from 'react';
import { inject, observer } from 'mobx-react';

@inject('store') @observer
export default class ManualCompetition extends Component {
  constructor(props) {
    super(props);
    this.state = {
      teamNumber: '',
      confirmNumber: false,
    };
    // 要求整个竞赛过程中一直打开 不然清空

    this.renderManualCompetition = this.renderManualCompetition.bind(this);
    this.handleInput = (event) => {
      this.setState({ teamNumber: event.target.value });
    };
    this.determinTeamNumber = this.determinTeamNumber.bind(this);
  }

  determinTeamNumber() {
    //Todo: check team number is valid
    this.props.store.hmi.setTeamNumber(this.state.teamNumber);
    this.setState({ confirmNumber: true });
  }

  renderManualCompetition() {
    const teamNumber = this.props.store.hmi.teamNumber;
    return (
      <div className="monitor-content">
        <div className="speed-section"></div>
        <div className="monitor-row section">
          <label className="one">当前车辆所在位置</label>
          <span className="two">position</span>
        </div>
        <div className="monitor-row section">
          <label className="one">超出赛道次数</label>
          <span className="two">position</span>
        </div>
        <div className="monitor-row foul-section">
          <label className="one">犯规次数（低于或高于限速范围都算犯规）</label>
          <span className="two">position</span>
        </div>
        <div className="monitor-row number-section">
          <label className="number-label">请输入参赛队伍编号</label>
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
