import React from "react";
import { inject, observer } from "mobx-react";

import { TELEOP_WS } from "store/websocket";

import CheckboxItem from "components/common/CheckboxItem";
import AudioControl from "components/TeleopMonitor/AudioControl";
import MonitorSection from "components/TeleopMonitor/MonitorSection";

import itemIcon from "assets/images/icons/teleop_item.png";

function OperationButton(props) {
    const { name, command } = props;

    return (
        <button
            className="command-button teleop-button"
            onClick={() => {
                if (confirm(`Are you sure you want to execute ${name} command?`)) {
                    command();
                }
            }}
        >
            {name}
        </button>
    );
}

@inject("store") @observer
export default class ConsoleTeleOp extends React.Component {
    constructor(props) {
        super(props);

        this.operation = {
            "STOP": () => {
                TELEOP_WS.executeCommand("EStop");
            },
            "PULL OVER": () => {
                TELEOP_WS.executeCommand("PullOver");
            },
            "RESUME": () => {
                TELEOP_WS.executeCommand("ResumeCruise");
            }
        };
    }

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
                    <div className="title">Console Teleop Controls</div>
                </div>

                <div className="monitor-content">
                    <AudioControl
                        id='console'
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
                    <MonitorSection title="Video" icon={itemIcon}>
                        <CheckboxItem id='teleopVideo'
                                      title={"On/Off"}
                                      isChecked={teleop.videoEnabled}
                                      onClick={() => {
                                        TELEOP_WS.executeCommand('ToggleVideo');
                                        teleop.toggleVideo();
                                      }} />
                    </MonitorSection>
                    <MonitorSection title="Signal" icon={itemIcon}>
                        {teleop.modemInfo.entries().map(([name, value]) => (
                                <div className="section-content-row" key={name}>
                                    <span className="field">{name}</span>
                                    <span className="field">{value}</span>
                                </div>
                            ))
                        }
                    </MonitorSection>
                    <MonitorSection title="Operation" icon={itemIcon}>
                        <div className="teleop-command-group">
                            {Object.entries(this.operation).map(([name, command]) => (
                                <OperationButton key={name} name={name} command={command} />)
                            )}
                        </div>
                    </MonitorSection>
                </div>
            </div >
        );
    }
}
