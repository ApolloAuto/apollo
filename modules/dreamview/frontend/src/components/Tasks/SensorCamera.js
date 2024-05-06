import React from 'react';
import { inject, observer } from 'mobx-react';
import { CAMERA_WS } from 'store/websocket';

import './style.scss';

export class CameraVideo extends React.Component {
  render() {
    return (
      <div className="camera-video">
        <img src={this.props.imageSrcData} alt={'camera sensor'} />
      </div>
    );
  }
}

@inject('store') @observer
export default class SensorCamera extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      channels: [],
    };
  }

  componentDidMount() {
    setTimeout(() => {
      CAMERA_WS
        .getCameraChannel().then((channels) => {
          this.setState({ channels });
        });
    }, 200);
  }

  onStatusSelectChange = (event) => {
    const value = event.target.value;
    if (value) {
      CAMERA_WS
        .stopCamera()
        .changeCameraChannel(value)
        .startCamera();
    }
  };

  render() {
    const { store } = this.props;
    const {
      cameraData,
      hmi,
    } = this.props.store;
    return (
      <div className="card camera">
        <div className="card-header"><span>Camera View</span>
          <span className='camera_view_channel_select'>
            <span className="arrow" />
            <select
              value={hmi.currentCameraSensorChannel}
              onChange={this.onStatusSelectChange}
            >
              <option key={'select—Ïchannel'} value={''}>- select channel -</option>
              {
                this.state.channels?.map((channel) => {
                  return (
                    <option key={channel} value={channel}>{channel}</option>
                  );
                })
              }
            </select>
          </span>
        </div>
        <div className="card-content-column">
          <CameraVideo imageSrcData={cameraData.imageSrcData} />
        </div>
      </div>
    );
  }
}
