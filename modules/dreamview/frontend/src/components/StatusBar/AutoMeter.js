import React from "react";
import { observer } from "mobx-react";

class Meter extends React.Component {
    render () {
        const { label, percentage, meterColor, background } = this.props;

        return (
            <div className="meter-container">
                <div className="meter-label">{label}</div>
                <span className="meter-head"
                      style={{borderColor: meterColor}}/>
                <div className="meter-background"
                     style={{backgroundColor: background}}>
                    <span style={{
                          backgroundColor: meterColor,
                          width: percentage + '%',
                    }}/>
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
                meterColor: '#b43131',
                background: '#382626',
            },
            accelerator: {
                label: 'Accelerator',
                meterColor: '#006aff',
                background: '#2d3b50',
            },
        };
    }

    render() {
        const { throttlePercent, brakePercent, speed } = this.props;

        return (
            <div className="auto-meter">
                <span className="speed-read">{speed}</span>
                <span className="speed-unit">km/h</span>
                <div className="brake-panel">
                    <Meter label={this.setting.brake.label}
                           percentage={brakePercent}
                           meterColor={this.setting.brake.meterColor}
                           background={this.setting.brake.background}/>
                </div>
                <div className="throttle-panel">
                    <Meter label={this.setting.accelerator.label}
                           percentage={throttlePercent}
                           meterColor={this.setting.accelerator.meterColor}
                           background={this.setting.accelerator.background}/>
                </div>
            </div>
        );
    }
}
