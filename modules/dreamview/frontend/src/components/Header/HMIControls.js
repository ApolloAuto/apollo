import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";

import HMISelectors from "components/Header/HMISelectors";

@inject("store") @observer
export default class HMIControls extends React.Component {
    render() {
        const {
            dockerImage,
            modes, currentMode,
            maps, currentMap,
            vehicles, currentVehicle,
            isCoDriver,
            isMute
        } = this.props.store.hmi;

        return (
            <React.Fragment>
                <button className="header-item header-button" onClick={() => alert(dockerImage)}>
                    Docker Version
                </button>
                <button
                    className={classNames({
                        "header-item": true,
                        "header-button": true,
                        "header-button-active": isCoDriver,
                    })}
                    onClick={() => this.props.store.hmi.toggleCoDriverFlag()}>
                    Co-Driver
                </button>
                <button
                    className={classNames({
                        "header-item": true,
                        "header-button": true,
                        "header-button-active": isMute,
                    })}
                    onClick={() => this.props.store.hmi.toggleMuteFlag()}>
                    Mute
                </button>
                <HMISelectors
                    modes={modes}
                    currentMode={currentMode}
                    maps={maps}
                    currentMap={currentMap}
                    vehicles={vehicles}
                    currentVehicle={currentVehicle} />
            </React.Fragment>
        );
    }
}
