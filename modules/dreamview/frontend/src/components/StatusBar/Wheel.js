import React from 'react';
import { observer } from 'mobx-react';

import classNames from 'classnames';

class Wheel extends React.PureComponent {
  render() {
    const { steeringAngle } = this.props;

    return (
            <svg className="wheel" viewBox="0 0 100 100" height="80" width="80">
                <circle className="wheel-background" cx="50" cy="50" r="45" />
                <g className="wheel-arm" transform={`rotate(${steeringAngle} 50 50)`}>
                    <rect x="45" y="7" height="10" width="10" />
                    <line x1="50" y1="50" x2="50" y2="5" />
                </g>
            </svg>
    );
  }
}

@observer
export default class WheelPanel extends React.Component {
  render() {
    const { steeringPercentage, steeringAngle, turnSignal } = this.props;

    const isLeftOn = (turnSignal === 'LEFT' || turnSignal === 'EMERGENCY');
    const isRightOn = (turnSignal === 'RIGHT' || turnSignal === 'EMERGENCY');

    return (
            <div className="wheel-panel">
                <div className="steerangle-read">{steeringPercentage}</div>
                <div className="steerangle-unit">%</div>
                <div className={classNames({ 'left-arrow': true, 'blink-left': isLeftOn })} />
                <Wheel steeringAngle={steeringAngle} />
                <div className={classNames({ 'right-arrow': true, 'blink-right': isRightOn })} />
            </div>
    );
  }
}
