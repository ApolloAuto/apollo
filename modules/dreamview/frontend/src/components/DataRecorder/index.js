import React from "react";

import STORE from "store";
import WS from "store/websocket";

import PortalModal from "components/common/PortalModal";

export default class DriveEventEditor extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            eventTime: new Date(),
            eventMessage: "",
            popupReminder: this.props.newDisengagementReminder,
        };

        this.handleMessageChange = this.handleMessageChange.bind(this);
        this.handleTimestampUpdate = this.handleTimestampUpdate.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
        this.handleCancel = this.handleCancel.bind(this);
        this.setTextareaRef = (element) => {
            this.textareaElement = element;
        };
    }

    componentWillReceiveProps(nextProps) {
        if (nextProps.newDisengagementReminder) {
            this.handleTimestampUpdate();
            this.setState({popupReminder: true});
        }
    }

    handleMessageChange(event) {
        this.setState({ eventMessage: event.target.value });
    }

    handleTimestampUpdate() {
        this.setState({ eventTime: new Date() });
    }

    handleSubmit() {
        if (this.state.eventMessage) {
            WS.submitDriveEvent(this.state.eventTime.getTime(), this.state.eventMessage);
            STORE.handleOptionToggle('showDataRecorder');
        } else {
            alert("Please provide a drive event message.");
        }
    }

    handleCancel() {
        STORE.handleOptionToggle('showDataRecorder');
    }

    render() {
        return (
            <div className="card data-recorder">
                <div className="card-header">
                    <span>Add Drive Event</span>
                </div>
                <div className="card-content-column">
                    <table>
                        <tbody>
                            <tr className="drive-event-time-row">
                                <td>Event Time</td>
                                <td>
                                    <span>
                                        {this.state.eventTime.toString()}
                                        <button
                                            className="timestamp-button"
                                            onClick={this.handleTimestampUpdate} >
                                            Update Time
                                        </button>
                                    </span>
                                </td>
                            </tr>
                            <tr className="drive-event-msg-row">
                                <td>Message</td>
                                <td>
                                    <textarea
                                        autoFocus={!this.state.popupReminder}
                                        ref={this.setTextareaRef}
                                        placeholder="please enter a message..."
                                        value={this.state.eventMessage}
                                        onChange={this.handleMessageChange} />
                                </td>
                            </tr>
                            <tr>
                                <td />
                                <td>
                                    <button className="submit-button" onClick={this.handleSubmit}>
                                        Submit
                                    </button>
                                    <button onClick={this.handleCancel}>
                                        Cancel
                                    </button>
                                </td>
                            </tr>
                        </tbody>
                    </table>
                </div>
                <PortalModal
                    open={this.state.popupReminder}
                    onClose={() => {
                        this.setState({popupReminder: false});
                        this.textareaElement.focus();
                    }} >
                    <div className="codriver-msg">
                        <p>Disengagement found. </p>
                        <p>Please record a drive event.</p>
                    </div>
                </PortalModal>
            </div>
        );
    }
}
