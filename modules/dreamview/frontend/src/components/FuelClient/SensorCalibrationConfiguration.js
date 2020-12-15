import React from 'react';
import { inject,observer } from 'mobx-react';

// the component show the lidar configure and camera configure
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
    //this.props.store.hmi.toggleTranslationChange();
    this.props.store.hmi.changeTranslation(this.props.belong,
      this.props.index, val, this.props.isLidar);
  }

  render() {
    return (
      <input
        className="translation-input"
        type="number"
        value={this.state.translationValue}
        onChange={this.handleTranslationChange}
      >
      </input>
    );
  }
}

@observer
export default class SensorCalibrationConfiguration extends React.Component {
  renderSensorConfiguration(sensorName, translation,isLidar) {
    return (
      <tr className="lidar-configuration-tr" key={sensorName}>
        <td className={(sensorName === this.props.mainSensor) ? 'main-sensor' : null}>{sensorName}</td>
        <td>
          <div className="lidar-configuration-translation">
            <div className="lidar-configuration-xyz">
              x:<TranslationInput belong={sensorName} index='x' value={translation.x} isLidar={isLidar}></TranslationInput>
            </div>
            <div className="lidar-configuration-xyz">y:<TranslationInput value={translation.y} belong={sensorName} index='y' isLidar={isLidar}></TranslationInput></div>
            <div className="lidar-configuration-xyz">z:<TranslationInput value={translation.z} index='z' belong={sensorName} isLidar={isLidar}></TranslationInput></div>
          </div>
        </td>
      </tr>
    );
  }

  render() {
    const { lidars, cameras } = this.props;
    const lidarConfigurations = [];
    lidars.forEach((trans, sensorName) => {
      lidarConfigurations.push(this.renderSensorConfiguration(sensorName, trans,true));
    });
    const cameraConfigurations = [];
    cameras.forEach((trans, cameraName) => {
      cameraConfigurations.push(this.renderSensorConfiguration(cameraName, trans,false));
    });
    return (
      <React.Fragment>
        <div className="lidar-configuration-table">
          {!_.isEmpty(cameraConfigurations) && (<div>
          <div>Camera Translation</div>
          <table>
              <tbody>
                 {cameraConfigurations}
              </tbody>
            </table>
          </div>)}
          <div>Lidar Translation</div>
          <table>
              <tbody>
                 {lidarConfigurations}
              </tbody>
          </table>
      </div>
  </React.Fragment>
    );
  }
}