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
    const val = parseFloat(event.target.value);
    this.setState({ translationValue: val });
    this.props.store.hmi.changeTranslation(
      this.props.belong,
      this.props.index,
      val,
      this.props.isLidar,
    );
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

@observer
export default class SensorCalibrationConfiguration extends React.Component {
  renderSensorConfiguration(sensorName, translation, isLidar) {
    return (
      <tr className="sensor-configuration-tr" key={sensorName}>
        <td
          className={
            sensorName === isLidar && this.props.mainSensor ? 'main-sensor' : null
          }
        >
          {sensorName}
        </td>
        <td>
          <div className="sensor-configuration-translation">
            <div className="sensor-configuration-xyz">
              x:
              <TranslationInput
                belong={sensorName}
                index="x"
                value={_.get(translation,'x')}
                isLidar={isLidar}
              ></TranslationInput>
            </div>
            <div className="sensor-configuration-xyz">
              y:
              <TranslationInput
                value={_.get(translation,'y')}
                belong={sensorName}
                index="y"
                isLidar={isLidar}
              ></TranslationInput>
            </div>
            <div className="sensor-configuration-xyz">
              z:
              <TranslationInput
                value={_.get(translation,'z')}
                index="z"
                belong={sensorName}
                isLidar={isLidar}
              ></TranslationInput>
            </div>
          </div>
        </td>
      </tr>
    );
  }

  render() {
    const { lidars, camera, componentStatus} = this.props;
    const lidarConfigurations = [];
    lidars.forEach((trans, sensorName) => {
      lidarConfigurations.push(
        this.renderSensorConfiguration(sensorName, trans, true),
      );
    });
    const cameraConfiguration = [];
    if (_.get(camera, 'translation')) {
      const translation = _.get(camera, 'translation');
      cameraConfiguration.push(
        this.renderSensorConfiguration('Lidar-Camera Translation', translation, false),
      );
    }
    return (
      <React.Fragment>
        <div className="sensor-configuration-table">
          {!_.isEmpty(cameraConfiguration) && (
            <div>
              <div>
                {_.get(
                  componentStatus.get('Camera'),
                  'message',
                )}
              </div>
              <table>
                <tbody>{cameraConfiguration}</tbody>
              </table>
            </div>
          )}
          {!_.isEmpty(lidarConfigurations) && (
            <div>
              <div>IMU-Lidar Translation</div>
              <table>
                <tbody>{lidarConfigurations}</tbody>
              </table>
            </div>
          )}
        </div>
      </React.Fragment>
    );
  }
}
