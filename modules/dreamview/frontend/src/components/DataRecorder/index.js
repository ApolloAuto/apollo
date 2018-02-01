import React from "react";
import { inject, observer } from "mobx-react";

import WS from "store/websocket";

class NewDriveEvent extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            event_msg: '',
        };

        this.handleChange = this.handleChange.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChange(event) {
        this.setState({event_msg: event.target.value});
    }

    handleSubmit() {
        const {event_time_ms, on_drive_event_submited} = this.props;
        WS.submitDriveEvent(event_time_ms, this.state.event_msg);
        on_drive_event_submited();
    }

    render() {
        const {event_time_ms} = this.props;
        return (
            <div className="card drive-event-card">
                <div className="card-header"><span>Adding New DriveEvent</span></div>
                <div className="card-content-column">
                    <table><tbody>
                        <tr><td>Event time</td><td>{event_time_ms}</td></tr>
                        <tr>
                            <td>Message</td>
                            <td>
                                <input type="text" className="drive-event-msg"
                                     value={this.state.event_msg}
                                     onChange={this.handleChange} />
                            </td>
                        </tr>
                    </tbody></table>
                    <button onClick={this.handleSubmit}>Submit</button>
                </div>
            </div>);
    }
}


@inject("store") @observer
export default class DataRecorder extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            newDriveEventTimeMs: 0,
            showDriveEvent: false,
        };
        this.handleNewDriveEvent = this.handleNewDriveEvent.bind(this);
        this.onDriveEventSubmited = this.onDriveEventSubmited.bind(this);
    }

    handleNewDriveEvent() {
        this.setState({
            newDriveEventTimeMs: new Date().getTime(),
            showDriveEvent: true,
        });
    }

    onDriveEventSubmited() {
        this.setState({
            showDriveEvent: false,
        });
    }

    render() {
        return (
            <div className="data-recorder">
              <div className="card">
                <div className="card-header"><span>Operations</span></div>
                <div className="card-content-column">
                  <button onClick={this.handleNewDriveEvent}>New DriveEvent</button>
                </div>
              </div>
              {this.state.showDriveEvent &&
                <NewDriveEvent event_time_ms={this.state.newDriveEventTimeMs}
                     on_drive_event_submited={this.onDriveEventSubmited}/>}
            </div>);
    }
}
