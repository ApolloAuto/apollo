import React from "react";
import protobuf from "protobufjs/light";
import classNames from "classnames";
import _ from "lodash";

import STORE from "store";
import WS from "store/websocket";
import PortalModal from "components/common/PortalModal";
import CheckboxItem from "components/common/CheckboxItem";

const simWorldRoot = protobuf.Root.fromJSON(require("proto_bundle/sim_world_proto_bundle.json"));
const DriveEventType = simWorldRoot.lookup("apollo.common.DriveEvent.Type").values;

export default class DriveEventEditor extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            eventTime: new Date(STORE.timestamp),
            eventMessage: "",
            eventTypes: new Set(),
            popupReminder: this.props.newDisengagementReminder,
            isReportable: false,
        };

        this.handleMessageChange = this.handleMessageChange.bind(this);
        this.handleTimestampUpdate = this.handleTimestampUpdate.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
        this.handleCancel = this.handleCancel.bind(this);
        this.toggleTicket = this.toggleTicket.bind(this);
        this.setTextareaRef = (element) => {
            this.textareaElement = element;
        };
    }

    componentWillReceiveProps(nextProps) {
        if (nextProps.newDisengagementReminder) {
            this.handleTimestampUpdate();
            this.setState({ popupReminder: true });
        }
    }

    handleMessageChange(event) {
        this.setState({ eventMessage: event.target.value });
    }

    handleTimestampUpdate() {
        this.setState({ eventTime: new Date(STORE.timestamp) });
    }

    handleSubmit(event) {
        event.preventDefault();
        if (!this.state.eventMessage) {
            return alert("Please provide a drive event message.");
        }
        if (!this.state.eventTypes.size) {
            return alert("Please select at least one event type.");
        }

        WS.submitDriveEvent(
            this.state.eventTime.getTime(),
            this.state.eventMessage,
            this.state.eventTypes,
            this.state.isReportable,
        );
        STORE.handleOptionToggle('showDataRecorder');
    }

    handleCancel() {
        STORE.handleOptionToggle('showDataRecorder');
    }

    handleEventTypeSelection(type) {
        const eventTypes = this.state.eventTypes;
        if (eventTypes.has(type)) {
            eventTypes.delete(type);
        } else {
            eventTypes.add(type);
        }
        this.setState({ eventTypes: new Set(eventTypes) });
    }

    toggleTicket() {
        this.setState((prevState) => {
            return { isReportable: !prevState.isReportable };
        });
    }

    renderTypeCheckBox() {
        const typeCheckbox = Object.keys(DriveEventType).map(type => {
            return (
                <button
                    key={type}
                    onClick={this.handleEventTypeSelection.bind(this, type)}
                    className={classNames({
                        "drive-event-type-button": true,
                        "drive-event-type-button-active": this.state.eventTypes.has(type),
                    })}
                >
                    {_.startCase(_.lowerCase(type))}
                </button>
            );
        });
        return typeCheckbox;
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
                            <tr className="drive-event-row">
                                <td>Event Time</td>
                                <td>
                                    <span className="event-time">
                                        {this.state.eventTime.toString()}
                                        <button
                                            className="timestamp-button"
                                            onClick={this.handleTimestampUpdate} >
                                            Update Time
                                        </button>
                                    </span>
                                </td>
                            </tr>
                            <tr className="drive-event-row">
                                <td>Types</td>
                                <td>{this.renderTypeCheckBox()}</td>
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
                            <tr className="drive-event-row">
                                <td>File a ticket</td>
                                <td className="multiple-items">
                                    <span>
                                        <CheckboxItem
                                            id="isReportableData"
                                            isChecked={this.state.isReportable}
                                            disabled={false}
                                            onClick={this.toggleTicket}
                                        />
                                    </span>
                                    <span>
                                        <button className="cancel-button"
                                                onClick={this.handleCancel}>
                                            Cancel
                                        </button>
                                        <button className="submit-button"
                                                onClick={this.handleSubmit}>
                                            Submit
                                        </button>
                                    </span>
                                </td>
                            </tr>
                        </tbody>
                    </table>
                </div>
                <PortalModal
                    open={this.state.popupReminder}
                    onClose={() => {
                        this.setState({ popupReminder: false });
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
