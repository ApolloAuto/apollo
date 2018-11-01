import React from "react";

import Selector from "components/Header/Selector";
import WS from "store/websocket";

export default class HMISelectors extends React.Component {
    render() {
        const { modes, currentMode,
                maps, currentMap,
                vehicles, currentVehicle } = this.props;

        return (
            <React.Fragment>
                <Selector name="setup mode"
                          options={modes}
                          currentOption={currentMode}
                          onChange={(event) => {
                            WS.changeSetupMode(event.target.value);
                          }} />
                <Selector name="vehicle"
                          options={vehicles}
                          currentOption={currentVehicle}
                          onChange={(event) => {
                            WS.changeVehicle(event.target.value);
                          }} />
                <Selector name="map"
                          options={maps}
                          currentOption={currentMap}
                          onChange={(event) => {
                            WS.changeMap(event.target.value);
                          }} />
            </React.Fragment>
        );
    }
}
