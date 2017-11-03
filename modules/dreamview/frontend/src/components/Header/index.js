import React from "react";
import { inject, observer } from "mobx-react";

import Image from "components/common/Image";
import logoApollo from "assets/images/logo_apollo.png";
import Selector from "components/Header/Selector";
import WS from "store/websocket";

@inject("store") @observer
export default class Header extends React.Component {
    render() {
        const { maps, vehicles, currentMap, currentVehicle } = this.props.store.hmi;

        return (
            <div className = "menu-bar">
                <Image image = {logoApollo} className="apollo-logo" />
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
            </div>
        );
    }
}