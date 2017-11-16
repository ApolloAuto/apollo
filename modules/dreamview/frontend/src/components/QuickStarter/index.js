import React from "react";

import WS from "store/websocket";

export default class QuickStarter extends React.Component {
    render() {
        return (
            <div className="quick-starter card">
                <div className="card-header"><span>Quick Start</span></div>
                <div className="card-content-column">
                    <div className="command-group">
                        <span className="name">Record:</span>
                        <button onClick={() => {
                            WS.executeToolCommand("rtk_record_replay", "setup");
                        }}>Setup</button>
                        <button onClick={() => {
                            WS.executeToolCommand("rtk_record_replay", "start_recorder");
                        }}>Start</button>
                        <button onClick={() => {
                            WS.executeToolCommand("rtk_record_replay", "stop_recorder");
                        }}>Stop</button>
                    </div>
                    <div className="command-group">
                        <span className="name">Replay:</span>
                        <button onClick={() => {
                            WS.executeToolCommand("rtk_record_replay", "setup");
                        }}>Setup</button>
                        <button onClick={() => {
                            WS.executeToolCommand("rtk_record_replay", "start_player");
                        }}>Start</button>
                        <button onClick={() => {
                            WS.executeToolCommand("rtk_record_replay", "stop_player");
                        }}>Stop</button>
                    </div>
                    <div className="command-group">
                        <span className="name">Auto:</span>
                        <button onClick={() => {
                            WS.executeModeCommand("start");
                        }}>Setup</button>
                        <button onClick={() => {
                            WS.changeDrivingMode("COMPLETE_AUTO_DRIVE");
                        }}>Start Auto</button>
                    </div>
                    <div className="command-group">
                        <span className="name">Reset:</span>
                        <button onClick={() => {
                            WS.executeModeCommand("stop");
                        }}>Reset All</button>
                    </div>
                </div>
            </div>
        );
    }
}