import React from 'react';
import { observer } from 'mobx-react';

@observer
export default class Electricity extends React.Component {

  render() {
    const {
      electricityPercentage,
    } = this.props;

    if (electricityPercentage === null) {
      return null;
    }

    const percentageString = `${electricityPercentage}%`;
    const rectWidth = 24 * (electricityPercentage / 100);
    const electricityColor = (electricityPercentage <= 20)
      ? 'rgba(180, 49, 49, 0.8)' : 'rgba(79, 198, 105, 0.8)';

    return (
            <div className="battery-and-gears electricity-status">
                <div className="left-div">
                    <svg version="1.1">
                        <rect rx="2" ry="2" x="44" y="10" width="30" height="15" fill="rgb(15,127,18)"/>
                        <rect x="46" y="12" width="26" height="11" fill="rgb(0,0,0,0.5)"/>
                        <rect x="47" y="13" width={ rectWidth } height="9" fill= { electricityColor }/>
                        <rect rx="2" ry="2" x="74" y="13" height="8" width="2" fill="rgb(15,127,18)"/>
                    </svg>
                </div>
                <div className="right-div">
                    <div className="text"> { percentageString } </div>
                </div>
            </div>
    );
  }
}
