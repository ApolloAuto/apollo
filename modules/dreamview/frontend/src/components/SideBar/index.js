import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import SecondaryButton from "components/SideBar/SecondaryButton";
import WS from "store/websocket";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly, hmi } = this.props.store;

        const settings = {};
        const optionNames = [...options.mainSideBarOptions, ...options.secondarySideBarOptions];
        optionNames.forEach(optionName => {
            settings[optionName] = {
                active: options[optionName],
                onClick: () => {
                    this.props.store.handleOptionToggle(optionName);
                },
                disabled: options.isSideBarButtonDisabled(
                    optionName,
                    enableHMIButtonsOnly,
                    hmi.inNavigationMode
                ),
            };
        });

        return (
            <div className="side-bar">
                <ButtonPanel settings={settings} />
                <div className="sub-button-panel">
                    <SecondaryButton
                        panelLabel="Voice Command"
                        disabled={settings.enableVoiceCommand.disabled}
                        onClick={settings.enableVoiceCommand.onClick}
                        active={settings.enableVoiceCommand.active} />
                    <SecondaryButton
                        panelLabel="Default Routing"
                        disabled={settings.showPOI.disabled}
                        onClick={settings.showPOI.onClick}
                        active={!options.showRouteEditingBar && options.showPOI} />
                </div>
            </div>
        );
    }
}
