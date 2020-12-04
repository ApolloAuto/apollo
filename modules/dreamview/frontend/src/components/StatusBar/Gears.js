import React from 'react';
import { observer } from 'mobx-react';

@observer
export default class Gears extends React.Component {

  render() {
    const {
      electricityPercentage,
      currentGear,
    } = this.props;

    const gearAlphabet = (currentGear && currentGear !== 'GEAR_NONE') ? currentGear.charAt(5) : 'None';
    const gearLabel = 'Gear';

    return (
            <div className="battery-and-gears" style={{ top: (electricityPercentage !== null) ? '328px' : '287px'}}>
                <div className="left-div">
                    <div className="text"> { gearLabel } </div>
                </div>
                <div className="right-div">
                    <div className="text"> { gearAlphabet } </div>
                </div>
            </div>
    );
  }
}

