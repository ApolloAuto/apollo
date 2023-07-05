import React from 'react';
import { inject, observer } from 'mobx-react';

const StatusColorMapping = {
  OK: '#1C9063',
  NOT_READY: '#B43131',
  NOT_PRESENT: '#B43131',
  ERR: '#B43131',
  UNDEF: '#B43131',
};

@observer
export default class StatusDisplay extends React.Component {
  constructor(props) {
    super(props);

    this.showStatusMessage = this.showStatusMessage.bind(this);
  }

  showStatusMessage() {
    if (this.props.status.message) {
      alert(`${this.props.title}: ${this.props.status.message}`);
    } else {
      alert(`No message from ${this.props.title}`);
    }
  }

  render() {
    const { title, status } = this.props;
    const status_code = status.status;

    return (
            <div className="status-display">
                <div className="name">{title}</div>
                <div className="status" onClick={this.showStatusMessage}>
                    <span>{status_code}</span>
                    <span
                        className="status-icon"
                        style={{
                          backgroundColor: StatusColorMapping[status_code],
                        }}
                    />
                </div>
            </div>
    );
  }
}
