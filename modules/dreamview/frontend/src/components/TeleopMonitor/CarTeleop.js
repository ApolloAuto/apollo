import React from "react";
import { inject, observer } from "mobx-react";

import { TELEOP_WS } from "store/websocket";

import AudioControl from "components/TeleopMonitor/AudioControl";
import MonitorSection from "components/TeleopMonitor/MonitorSection";

import itemIcon from "assets/images/icons/teleop_item.png";


@inject("store") @observer
export default class CarTeleOp extends React.Component {
    componentDidMount() {
        TELEOP_WS.initialize();
    }

    componentWillUnmount() {
        TELEOP_WS.close();
    }

    render() {
        const { teleop } = this.props.store;

        return (
            <div className="monitor teleop">
                <div className="monitor-header">
                    <div className="title">Car Teleop Controls</div>
                </div>

                <div className="monitor-content">
                    <AudioControl id='car'
                                  audioEnabled={teleop.audioEnabled}
                                  micEnabled={teleop.micEnabled}
                                  toggleAudio={() => {
                                    TELEOP_WS.executeCommand('ToggleAudio');
                                    teleop.toggleAudio();
                                  }}
                                  toggleMic={() => {
                                    teleop.toggleMic();
                                    TELEOP_WS.executeCommand('ToggleMic');
                                  }} />
                    <MonitorSection title="Signal" icon={itemIcon}>
                        {teleop.modemInfo.entries().map(([name, value]) => (
                                <div className="section-content-row" key={name}>
                                    <span className="field">{name}</span>
                                    <span className="field">{value}</span>
                                </div>
                            ))
                        }
                    </MonitorSection>
                </div>
            </div >
        );
    }
}