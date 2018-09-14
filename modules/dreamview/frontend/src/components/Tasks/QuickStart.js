import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";

import UTTERANCE from "store/utterance";
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
                UTTERANCE.speakOnce('Start RTK recorder');
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_recorder");
                UTTERANCE.speakOnce('Stop RTK recorder');
            },
        };

        this.rtkReplay = {
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
                UTTERANCE.speakOnce('Start RTK replay');
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_player");
                UTTERANCE.speakOnce('Stop RTK replay');
            },
        };

        this.setup = {
            "Setup": () => {
                WS.executeModeCommand("start");
                UTTERANCE.speakOnce('Setup');
            },
        };

        this.reset = {
            "Reset All": () => {
                WS.executeModeCommand("stop");
                UTTERANCE.speakOnce('Reset All');
            },
        };

        this.auto = {
            "Start Auto": () => {
                WS.changeDrivingMode("COMPLETE_AUTO_DRIVE");
                UTTERANCE.speakOnce('Start Auto');
            },
        };
    }

    componentWillUpdate() {
        UTTERANCE.cancelAllInQueue();
    }

    render() {
        const { hmi } = this.props.store;
        const { tasksPanelLocked } = this.props.store.options;

        return (
            <div className="card">
                <div className="card-header"><span>Quick Start</span></div>
                <div className="card-content-column">
                    <CommandGroup disabled={tasksPanelLocked} commands={this.setup} />
                    <CommandGroup disabled={tasksPanelLocked} commands={this.reset} />
                    <CommandGroup disabled={!hmi.enableStartAuto || tasksPanelLocked}
                                  commands={this.auto}
                                  extraButtonClass="start-auto-button"
                                  extraCommandClass="start-auto-command" />
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Record"
                                      disabled={tasksPanelLocked}
                                      commands={this.rtKRecord} />}
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Replay"
                                      disabled={tasksPanelLocked}
                                      commands={this.rtkReplay} />}
                </div>
            </div>
        );
    }
}