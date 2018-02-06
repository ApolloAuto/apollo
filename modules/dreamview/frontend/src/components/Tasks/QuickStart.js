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
                const msg = new SpeechSynthesisUtterance('Start RTK recorder');
                window.speechSynthesis.speak(msg);
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_recorder");
                const msg = new SpeechSynthesisUtterance('Stop RTK recorder');
                window.speechSynthesis.speak(msg);
            },
        };

        this.rtkReplay = {
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
                const msg = new SpeechSynthesisUtterance('Start RTK replay');
                window.speechSynthesis.speak(msg);
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_player");
                const msg = new SpeechSynthesisUtterance('Stop RTK replay');
                window.speechSynthesis.speak(msg);
            },
        };

        this.setup = {
            "Setup": () => {
                WS.executeModeCommand("start");
                const msg = new SpeechSynthesisUtterance('Setup');
                window.speechSynthesis.speak(msg);
            },
        };

        this.reset = {
            "Reset All": () => {
                WS.executeModeCommand("stop");
                const msg = new SpeechSynthesisUtterance('Reset All');
                window.speechSynthesis.speak(msg);
            },
        };

        this.auto = {
            "Start Auto": () => {
                WS.changeDrivingMode("COMPLETE_AUTO_DRIVE");
                const msg = new SpeechSynthesisUtterance('Start Auto');
                window.speechSynthesis.speak(msg);
            },
        };
        this.version = {
            "Version": () => {
                // TODO(all): change to nice UI.
                alert(this.props.store.hmi.dockerImage);
            },
        };
    }

    render() {
        const { hmi } = this.props.store;
        const { tasksPanelLocked } = this.props.store.options;

        return (
            <div className="card">
                <div className="card-header"><span>Quick Start</span></div>
                <div className="card-content-column">
                    <CommandGroup disabled={false} commands={this.version} />
                    <CommandGroup disabled={tasksPanelLocked} commands={this.setup} />
                    <CommandGroup disabled={tasksPanelLocked} commands={this.reset} />
                    <CommandGroup disabled={!hmi.enableStartAuto} commands={this.auto}
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