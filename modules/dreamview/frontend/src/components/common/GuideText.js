import React from 'react';
import lidarImuIMG from 'assets/images/lidar_imu.png';
import cameraLidarPositionIMG from 'assets/images/camera_lidar_position.png';

// For sensor calibration mode instruction
export default class GuideText extends React.Component {
  render() {
    const { mode } = this.props;
    switch (mode) {
      case 'Lidar-IMU Sensor Calibration':
        return (<div className="guide-text">
        <p className="pane-text">
          Start recorder while localization and lidar component status are green.
          Drive the vehicle following a ∞ symbol path at a speed within 1m/s,
          and keep the turning radius as small as possible.
          The total time length should be within 3 minutes,
          and the drive should contain at least 2 full ∞ symbol paths.
        </p>
        <img className="pane-img" src={lidarImuIMG}></img>
      </div>);
      case 'Camera-Lidar Sensor Calibration':
        return (<div className="guide-text">
          <p className="pane-text">
            Start recorder while GNSS、localization and lidar component status are green.
            During recording, drive slowly in a straight line for 10 seconds
            and then stop for 5 seconds. Repeat this 5 times.
          </p>
          <p className="pane-text">
          Sensor coordinate system picture
          </p>
          <img className="pane-img" src={cameraLidarPositionIMG}></img>
        </div>);
      default:
        return null;
    }
  }
}
