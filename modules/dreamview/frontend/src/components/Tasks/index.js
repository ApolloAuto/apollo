import React from "react";

import QuickStart from "components/Tasks/QuickStart";
import Others from "components/Tasks/Others";
import Delay from "components/Tasks/Delay";
import Console from "components/Tasks/Console";

export default class Tasks extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            isPanelLocked: false,
        };

        this.toggleLock = this.toggleLock.bind(this);
    }

    toggleLock() {
        this.setState({
          isPanelLocked: !this.state.isPanelLocked,
        });
    }

    render() {
        return (
            <div className="tasks">
                <QuickStart isPanelLocked={this.state.isPanelLocked} />
                <Others toggleLock={this.toggleLock}
                        isPanelLocked={this.state.isPanelLocked} />
                <Delay />
                <Console />
            </div>
        );
    }
}