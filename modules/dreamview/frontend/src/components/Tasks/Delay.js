import React from "react";
import { inject, observer } from "mobx-react";

import { millisecondsToTime } from "utils/misc";

class Delay extends React.Component {
    render() {
        const { time } = this.props;

        const timeString = (time === '-') ? time : millisecondsToTime(time | 0);

        return (
            <div className="value">{timeString}</div>
        );
    }
}


@inject("store") @observer
export default class DelayTable extends React.Component {
    render() {
        const { moduleDelay } = this.props.store;

        const items = moduleDelay.keys()
            .map(key => {
                const module = moduleDelay.get(key);

                return (
                    <div className="delay-item" key={'delay_' + key}>
                        <div className="name">{module.name}</div>
                        <Delay time={module.delay} />
                    </div>
                );
            });

        return (
            <div className="delay card">
                <div className="card-header"><span>Module Delay</span></div>
                <div className="card-content-column">
                    {items}
                </div>
            </div>
        );
    }
}