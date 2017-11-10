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
        const { routeEditingManager } = this.props.store;

        return (
            <header className = "header">
                <Image image={logoApollo} className="apollo-logo" />
                <Selector name="setup mode"
                          options={modes}
                          currentOption={currentMode}
                          onChange={(event) => {
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
                <Selector name="point of interest"
                          options={Object.keys(routeEditingManager.defaultRoutingEndPoint)}
                          currentOption={routeEditingManager.currentPOI}
                          onChange={(event) => {
                            routeEditingManager.setDefaultEndPoint(event.target.value);
                          }}/>
            </header>
        );
    }
}