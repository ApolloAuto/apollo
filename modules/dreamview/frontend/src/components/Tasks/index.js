import React from "react";

import QuickStart from "components/Tasks/QuickStart";
import Others from "components/Tasks/Others";
import Delay from "components/Tasks/Delay";
import Console from "components/Tasks/Console";

export default class Tasks extends React.Component {
    render() {
        return (
            <div className="tasks">
                <QuickStart />
                <Others />
                <Delay />
                <Console />
            </div>
        );
    }
}