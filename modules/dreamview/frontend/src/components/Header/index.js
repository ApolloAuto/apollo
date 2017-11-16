import React from "react";
import { inject, observer } from "mobx-react";

import Image from "components/common/Image";
import Selector from "components/Header/Selector";
import WS from "store/websocket";
import logoApollo from "assets/images/logo_apollo.png";


@inject("store") @observer
export default class Header extends React.Component {
    render() {
        const { modes, currentMode,
                maps, currentMap,
                vehicles, currentVehicle } = this.props.store.hmi;

        return (
            <header className = "header">
                <Image image={logoApollo} className="apollo-logo" />
                <Selector name="setup mode"
                          options={Object.keys(modes).sort()}
                          currentOption={currentMode}
                          onChange={(event) => {
                            this.props.store.hmi.currentMode = event.target.value;
                            WS.changeSetupMode(event.target.value);
                          }}/>
                <Selector name="vehicle"
                          options={vehicles}
                          currentOption={currentVehicle}
                          onChange={(event) => {
                            WS.changeVehicle(event.target.value);
                          }}/>
                <Selector name="map"
                          options={maps}
                          currentOption={currentMap}
                          onChange={(event) => {
                            WS.changeMap(event.target.value);
                          }}/>
            </header>
        );
    }
}