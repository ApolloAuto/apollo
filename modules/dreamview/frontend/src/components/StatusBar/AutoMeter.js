import React from 'react';
import { observer } from 'mobx-react';

import Speedometer from 'components/StatusBar/Speedometer';

class Meter extends React.Component {
  render() {
    const {
      label, percentage, meterColor, background,
    } = this.props;

    const percentageString = `${percentage}%`;

    return (
            <div className="meter-container">
                <div className="meter-label">
                    {label}
                    <span className="meter-value">{percentageString}</span>
                </div>
                <span
                    className="meter-head"
                    style={{ borderColor: meterColor }}
                />
                <div
                    className="meter-background"
                    style={{ backgroundColor: background }}
                >
                    <span style={{
                      backgroundColor: meterColor,
                      width: percentageString,
                    }}
                    />
                </div>
            </div>
    );
  }
}

@observer
export default class AutoMeter extends React.Component {
  constructor(props) {
    super(props);
    this.setting = {
      brake: {
        label: 'Brake',
        meterColor: '#B43131',
        background: '#382626',
      },
      accelerator: {
        label: 'Accelerator',
        meterColor: '#006AFF',
        background: '#2D3B50',
      },
    };
  }

  render() {
    const { throttlePercent, brakePercent, speed } = this.props;

    return (
            <div className="auto-meter">
                <Speedometer meterPerSecond={speed} />
                <div className="brake-panel">
                    <Meter
                        label={this.setting.brake.label}
                        percentage={brakePercent}
                        meterColor={this.setting.brake.meterColor}
                        background={this.setting.brake.background}
                    />
                </div>
                <div className="throttle-panel">
                    <Meter
                        label={this.setting.accelerator.label}
                        percentage={throttlePercent}
                        meterColor={this.setting.accelerator.meterColor}
                        background={this.setting.accelerator.background}
                    />
                </div>
            </div>
    );
  }
}
