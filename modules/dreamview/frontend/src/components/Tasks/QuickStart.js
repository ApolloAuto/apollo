import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";

import WS from "store/websocket";

class CommandGroup extends React.Component {
    render() {
        const { name, commands,disabled,
                extraCommandClass, extraButtonClass} = this.props;

        const entries = Object.keys(commands).map((key) => {
            return <button className={extraButtonClass}
                           disabled={disabled}
                           key={key}
                           onClick={commands[key]}>{key}</button>;
        });

        const text = name ? <span className="name">{`${name}:`}</span> : null;

        return (
            <div className={classNames("command-group", extraCommandClass)}>
                {text}
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
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_recorder");
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_recorder");
            },
        };

        this.rtkReplay = {
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
            },
        };

        this.setup = {
            "Setup": () => {
                WS.executeModeCommand("start");
            },
        };

        this.reset = {
            "Reset All": () => {
                WS.executeModeCommand("stop");
            },
        };

        this.auto = {
            "Start Auto": () => {
                WS.changeDrivingMode("COMPLETE_AUTO_DRIVE");
            },
        };
    }

    render() {
        const { hmi } = this.props.store;
        const { isPanelLocked } = this.props;

        return (
            <div className="card">
                <div className="card-header"><span>Quick Start</span></div>
                <div className="card-content-column">
                    <CommandGroup disabled={isPanelLocked} commands={this.setup} />
                    <CommandGroup disabled={isPanelLocked} commands={this.reset} />
                    <CommandGroup disabled={false} commands={this.auto}
                                  extraButtonClass="start-auto-button"
                                  extraCommandClass="start-auto-command" />
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Record"
                                      disabled={isPanelLocked}
                                      commands={this.rtKRecord} />}
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Replay"
                                      disabled={isPanelLocked}
                                      commands={this.rtkReplay} />}
                </div>
            </div>
        );
    }
}