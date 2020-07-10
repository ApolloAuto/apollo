import React from 'react';
import classNames from 'classnames';

import UTTERANCE from 'store/utterance';

export default class DrivingMode extends React.PureComponent {
  componentWillUpdate() {
    UTTERANCE.cancelAllInQueue();
  }

  render() {
    const { drivingMode, isAutoMode } = this.props;

    UTTERANCE.speakOnce(`Entering to ${drivingMode} mode`);

    return (
            <div className={classNames({
              'driving-mode': true,
              'auto-mode': isAutoMode,
              'manual-mode': !isAutoMode,
            })}
            >
                <span className="text">{drivingMode}</span>
            </div>
    );
  }
}
