import React from 'react';
import protobuf from 'protobufjs/light';
import classNames from 'classnames';
import _ from 'lodash';

import STORE from 'store';
import WS from 'store/websocket';
import CheckboxItem from 'components/common/CheckboxItem';

const simWorldRoot = protobuf.Root.fromJSON(require('proto_bundle/sim_world_proto_bundle.json'));

const AUDIO_TYPE = simWorldRoot.lookup('apollo.audio.AudioType').values;
const AUDIO_DIRECTION = simWorldRoot.lookup('apollo.audio.AudioDirection').values;
const MOVING_RESULT = simWorldRoot.lookup('apollo.audio.MovingResult').values;

export default class DriveEventEditor extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      eventTime: new Date(STORE.timestamp),
      obstacleId: '',
      audioType: null,
      movingResult: null,
      direction: null,
      isSirenOn: false,
    };

    this.handleObstacleIdChange = this.handleObstacleIdChange.bind(this);
    this.handleTimestampUpdate = this.handleTimestampUpdate.bind(this);
    this.handleSubmit = this.handleSubmit.bind(this);
    this.handleCancel = this.handleCancel.bind(this);
    this.toggleSiren = this.toggleSiren.bind(this);
  }

  handleObstacleIdChange(event) {
    this.setState({ obstacleId: event.target.value });
  }

  handleTimestampUpdate() {
    this.setState({ eventTime: new Date(STORE.timestamp) });
  }

  handleSubmit(event) {
    event.preventDefault();
    if (!this.state.obstacleId) {
      return alert('Missing obstacle ID.');
    }
    if (!this.state.audioType) {
      return alert('Please select an audio type.');
    }
    if (!this.state.movingResult) {
      return alert('Please select a moving result.');
    }
    if (!this.state.direction) {
      return alert('Please select a direction.');
    }
    const obstacleId = parseInt(this.state.obstacleId, 10);
    if (isNaN(obstacleId)) {
      return alert('Please enter a valid obstacle ID.');
    }

    WS.submitAudioEvent(
      this.state.eventTime.getTime(),
      obstacleId,
      AUDIO_TYPE[this.state.audioType],
      MOVING_RESULT[this.state.movingResult],
      AUDIO_DIRECTION[this.state.direction],
      this.state.isSirenOn,
    );
    STORE.handleOptionToggle('showDataRecorder');
  }

  handleCancel() {
    STORE.handleOptionToggle('showDataRecorder');
  }

  handleRadioSelection(itemName, value) {
    this.setState({ [itemName]: value });
  }

  toggleSiren() {
    this.setState((prevState) => ({ isSirenOn: !prevState.isSirenOn }));
  }

  renderRadioGroupButtons(groupName, items) {
    return items.sort().map((type) => (
      <button
        key={type}
        onClick={this.handleRadioSelection.bind(this, groupName, type)}
        className={classNames({
          'drive-event-type-button': true,
          'drive-event-type-button-active': this.state[groupName] === type,
        })}
      >
        {_.startCase(_.lowerCase(type))}
      </button>
    ));
  }

  render() {
    return (
      <React.Fragment>
        <div className="card-content-column">
          <table>
            <tbody>
              <tr className="drive-event-row">
                <td>Event Time</td>
                <td>
                  <span className="event-time">
                    {this.state.eventTime.toString()}
                    <button
                      className="timestamp-button"
                      onClick={this.handleTimestampUpdate}
                    >
                      Update Time
                    </button>
                  </span>
                </td>
              </tr>
              <tr className="drive-event-msg-row">
                <td>Obstacle ID</td>
                <td>
                  <input
                    autoFocus
                    placeholder="please enter an obstacle ID..."
                    value={this.state.obstacleId}
                    onChange={this.handleObstacleIdChange}
                  />
                </td>
              </tr>
              <tr className="drive-event-row">
                <td>Audio Type</td>
                <td>{this.renderRadioGroupButtons('audioType', Object.keys(AUDIO_TYPE))}</td>
              </tr>
              <tr className="drive-event-row">
                <td>Direction</td>
                <td>{this.renderRadioGroupButtons('direction', Object.keys(AUDIO_DIRECTION))}</td>
              </tr>
              <tr className="drive-event-row">
                <td>Moving Result</td>
                <td>{this.renderRadioGroupButtons('movingResult', Object.keys(MOVING_RESULT))}</td>
              </tr>
              <tr className="drive-event-row">
                <td>Is Siren On</td>
                <td className="multiple-items">
                  <span>
                    <CheckboxItem
                      id="isSirenOn"
                      isChecked={this.state.isSirenOn}
                      onClick={this.toggleSiren}
                    />
                  </span>
                  <span>
                    <button
                      className="cancel-button"
                      onClick={this.handleCancel}
                    >
                      Cancel
                    </button>
                    <button
                      className="submit-button"
                      onClick={this.handleSubmit}
                    >
                      Submit
                    </button>
                  </span>
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </React.Fragment>
    );
  }
}
