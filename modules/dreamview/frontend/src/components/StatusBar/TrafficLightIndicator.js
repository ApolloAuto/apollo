import React from 'react';

const ColorCodeMapping = {
  GREEN: 'rgba(79, 198, 105, 0.8)',
  YELLOW: 'rgba(239, 255, 0, 0.8)',
  RED: 'rgba(180, 49, 49, 0.8)',
  BLACK: 'rgba(30, 30, 30, 0.8)',
  UNKNOWN: 'rgba(30, 30, 30, 0.8)',
  '': null,
};

export default class TrafficLightIndicator extends React.PureComponent {
  render() {
    const { colorName } = this.props;

    const color = ColorCodeMapping[colorName];
    const text = colorName || 'NO SIGNAL';

    return (
            <div className="traffic-light">
                { color
                    && (
                        <svg className="symbol" viewBox="0 0 30 30" height="28" width="28">
                            <circle cx="15" cy="15" r="15" fill={color} />
                        </svg>
                    )}
                <div className="text">{text}</div>
            </div>
    );
  }
}
