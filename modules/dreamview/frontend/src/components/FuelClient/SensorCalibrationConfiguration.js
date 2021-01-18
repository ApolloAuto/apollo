import React from 'react';
import { inject, observer } from 'mobx-react';
import _ from 'lodash';

// The component show the lidar configure and camera configure
@inject('store') @observer
class TranslationInput extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      translationValue: props.value,
    };
    this.handleTranslationChange = this.handleTranslationChange.bind(this);
  }

  handleTranslationChange(event) {
    this.setState({ translationValue: event.target.value });
    const val = parseFloat(event.target.value);
    if (!isNaN(val)) {
      this.props.store.hmi.changeTranslation(
        this.props.belong,
        this.props.index,
        val,
        this.props.isLidar,
      );
    }
  }

  render() {
    return (
      <input
        className="translation-input"
        type="number"
        value={this.state.translationValue}
        onChange={this.handleTranslationChange}
      ></input>
    );
  }
}

@inject('store') @observer
class IntrinsicInput extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      intrinsicValue: props.value,
    };
    this.handleIntrinsicChange = this.handleIntrinsicChange.bind(this);
  }

  handleIntrinsicChange(event) {
    this.setState({ intrinsicValue: event.target.value });
    const val = parseFloat(event.target.value);
    if (!isNaN(val)) {
      this.props.store.hmi.changeIntrinsic(
        this.props.belong,
        this.props.index,
        val,
      );
    }
  }

  render() {
    return (
      <input
        className="camera-internal-configuration-d"
        type="number"
        value={this.state.intrinsicValue}
        onChange={this.handleIntrinsicChange}
      ></input>
    );
  }
}

@observer
export default class SensorCalibrationConfiguration extends React.Component {
  renderTranslation(sensorName, translation, isLidar) {
    return (
      <div className="sensor-configuration-translation">
        <div className="sensor-configuration-xyz">
          x:
          <br />
          <TranslationInput
            belong={sensorName}
            index='x'
            value={_.get(translation, 'x')}
            isLidar={isLidar}
          ></TranslationInput>
        </div>
        <div className="sensor-configuration-xyz">
          y:
          <br />
          <TranslationInput
            value={_.get(translation, 'y')}
            belong={sensorName}
            index='y'
            isLidar={isLidar}
          ></TranslationInput>
        </div>
        <div className="sensor-configuration-xyz">
          z:
          <br />
          <TranslationInput
            value={_.get(translation, 'z')}
            index='z'
            belong={sensorName}
            isLidar={isLidar}
          ></TranslationInput>
        </div>
      </div>
    );
  }

  renderLidarConfiguration(sensorName, translation) {
    return (
      <div className="sensor-configuration-tr" key={sensorName}>
        <div
          className={
            sensorName === this.props.mainSensor ? 'main-sensor' : null
          }
        >
          {sensorName}
        </div>
        <div>
          {this.renderTranslation(sensorName, translation, true)}
        </div>
      </div>
    );
  }

  renderCameraIntrinsics(name, params) {
    const result = [];
    const len = params.length;
    for (let i = 0; i < len; i++) {
      result.push(
        <div className="camera-internal-configuration-div">
          <div>{i + 1}:</div>
          <IntrinsicInput
            belong={name}
            index={i}
            value={params[i]}
          ></IntrinsicInput>
        </div>
      );
    }
    return result;
  }

  renderCameraConfiguration(sensorName, translation, D, K) {
    return (
      <div>
        <div className="sensor-configuration-tr" key={sensorName}>
          <div className={null}>
            {sensorName}
          </div>
          <div>
            {this.renderTranslation(sensorName, translation, false)}
          </div>
        </div>
        <div className="camera-intrinsics">
          Camera Intrinsics
          <div className="camera-intrinsics-D">
            D:
            <br />
            {this.renderCameraIntrinsics('D', D)}
          </div>
          <br />
          <div className="camera-intrinsics-K">
            K:
            <br />
            {this.renderCameraIntrinsics('K', K)}
          </div>
        </div>
      </div>
    );
  }

  render() {
    const { lidars, camera, componentStatus } = this.props;
    const lidarConfigurations = [];
    lidars.forEach((trans, sensorName) => {
      lidarConfigurations.push(
        this.renderLidarConfiguration(sensorName, trans, true)
      );
    });
    const cameraConfiguration = [];
    if (_.get(camera, 'translation')) {
      const translation = _.get(camera, 'translation');
      const D = _.get(camera, 'D');
      const K = _.get(camera, 'K');
      cameraConfiguration.push(
        this.renderCameraConfiguration('Lidar-Camera Translation', translation, D, K),
      );
    }
    return (
      <React.Fragment>
        <div className="sensor-configuration-table">
          {!_.isEmpty(cameraConfiguration) && (
            <div>
              <div className="camera-message">
                {_.get(
                  componentStatus.get('Camera'),
                  'message',
                )}
              </div>
              <table>
                <tbody>
                  <tr>
                    <td>{cameraConfiguration}</td>
                  </tr>
                </tbody>
              </table>
            </div>
          )}
          {!_.isEmpty(lidarConfigurations) && (
            <div>
              <div>IMU-Lidar Translation</div>
              <table>
                <tbody>
                  <tr>
                    <td>{lidarConfigurations}</td>
                  </tr>
                </tbody>
              </table>
            </div>
          )}
        </div>
      </React.Fragment>
    );
  }
}
