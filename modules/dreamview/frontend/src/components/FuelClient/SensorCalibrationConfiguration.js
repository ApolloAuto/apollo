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

//待优化 是inject store还是props 传递
@inject('store') @observer
export default class SensorCalibrationConfiguration extends React.Component {
  renderSensorConfiguration(sensorName, translation) {
    return (
      <tr className="lidar-configuration-tr" key={sensorName}>
        <td className={(sensorName === this.props.mainSensor) ? 'main-sensor' : null}>{sensorName}</td>
        <td>
          <div className="lidar-configuration-translation">
            <div className="lidar-configuration-xyz">
              x:<TranslationInput belong={sensorName} index='x' value={translation.x} isLidar={true}></TranslationInput>
            </div>
            <div className="lidar-configuration-xyz">y:<TranslationInput value={translation.y} belong={sensorName} index='y' isLidar={true}></TranslationInput></div>
            <div className="lidar-configuration-xyz">z:<TranslationInput value={translation.z} index='z' belong={sensorName} isLidar={true}></TranslationInput></div>
          </div>
        </td>
      </tr>
    );
  }

  render() {
    const { lidars, camera } = this.props;
    const lidarConfigurations = [];
    lidars.forEach((trans, sensorName) => {
      lidarConfigurations.push(this.renderSensorConfiguration(sensorName, trans));
    });
    const cameraConfiguration = [];
    if (_.get(camera, 'translation')) {
      const translation = _.get(camera, 'translation');
      cameraConfiguration.push(
        <tr className="lidar-configuration-tr" key='camera'>
          <td>{_.get(this.props.store.hmi.componentStatus.get('Camera'),'message')}</td>
        <td>
          <div className="lidar-configuration-translation">
            <div className="lidar-configuration-xyz">
                x:<TranslationInput belong={'camera'} index='x' value={_.get(translation, 'x')} isLidar={false}></TranslationInput>
            </div>
            <div className="lidar-configuration-xyz">y:<TranslationInput value={_.get(translation, 'y')} belong={'camera'} index='y' isLidar={false}></TranslationInput></div>
            <div className="lidar-configuration-xyz">z:<TranslationInput value={_.get(translation, 'z')} index='z' belong={'camera'} isLidar={false}></TranslationInput></div>
          </div>
        </td>
      </tr>
      );
    }
    return (
      <React.Fragment>
        <div className="lidar-configuration-table">
          {!_.isEmpty(cameraConfiguration) &&
          (<table>
              <tbody>
                 {cameraConfiguration}
              </tbody>
            </table>)}
          {!_.isEmpty(lidarConfigurations) && (<div>
            <div>Lidar Translation</div>
            <table>
              <tbody>
                {lidarConfigurations}
              </tbody>
            </table>
          </div>)}
      </div>
  </React.Fragment>
    );
  }
}
