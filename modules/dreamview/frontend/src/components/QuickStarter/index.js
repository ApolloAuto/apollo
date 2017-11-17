import React from "react";
import { inject, observer } from "mobx-react";

import WS from "store/websocket";

class CommandGroup extends React.Component {
    render() {
        const {name, commands} = this.props;

        const entries = Object.keys(commands).map((key) => {
            return <button key={key} onClick={commands[key]}>{key}</button>;
        });

        return (
            <div className="command-group">
                <span className="name">{name}:</span>
                {entries}
            </div>
        );
    }
}

@inject("store") @observer
export default class QuickStarter extends React.Component {
    constructor(props) {
        super(props);

        this.rtKRecord = {
            "Setup": () => {
                WS.executeToolCommand("rtk_record_replay", "setup");
            },
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_recorder");
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_recorder");
            },
        };

        this.rtkReplay = {
            "Setup": () => {
                WS.executeToolCommand("rtk_record_replay", "setup");
            },
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
            },
        };

        this.auto = {
            "Setup": () => {
                WS.executeModeCommand("start");
            },
            "Start Auto": () => {
                WS.changeDrivingMode("COMPLETE_AUTO_DRIVE");
            },
        };

        this.reset = {
            "Reset All": () => {
                WS.executeModeCommand("stop");
            },
        };
    }

    render() {
        const {hmi} = this.props.store;

        return (
            <div className="quick-starter card">
                <div className="card-header"><span>Quick Start</span></div>
                <div className="card-content-column">
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Record" commands={this.rtKRecord} />}
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Replay" commands={this.rtkReplay} />}
                    <CommandGroup name="Auto" commands={this.auto} />
                    <CommandGroup name="Reset" commands={this.reset} />
                </div>
            </div>
        );
    }
}