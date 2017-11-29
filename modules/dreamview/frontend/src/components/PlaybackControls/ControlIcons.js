import React from "react";

class PlayIcons extends React.Component {
    render() {
        const { onClick } = this.props;

        return (
            <svg className="icon" viewBox="0 0 10 10" onClick={onClick}>
                <polygon className="play" points="0 0, 10 5, 0 10" />
            </svg>
        );
    }
}


class ReplayIcons extends React.Component {
    render() {
        const { onClick } = this.props;

        return (
            <svg className="icon" viewBox="0 0 100 100" onClick={onClick}>
                 <path className="replay"
                       d="M0.166,49.438C0.166,75.152,20.744,96,46.125,96
                         c12.096,0,23.1-4.731,31.306-12.469
                         c2.144-2.021-8.776-12.227-10.269-10.84
                         c-5.54,5.146-12.926,8.286-21.037,8.286
                         c-17.193,0-31.133-14.122-31.133-31.544s13.939-31.545,31.133-31.545
                         c17.197,0,31.135,11.108,31.135,28.5
                         c0,0.007,0.021,0.062,0.049,0.069
                         L75.778,48c-3.484,0-5.931,0-5.931,0l14.826,18.023
                         L99.5,48
                         c0,0-2.447,0-5.931,0l-1.531-1.514
                         c0.017-0.006,0.05-0.015,0.05-0.021
                         c0-25.716-20.578-43.574-45.963-43.574
                         C20.744,2.891,0.166,23.723,0.166,49.438
                         z
                         M78.743,44.933
                         l0.115,0.023l-0.089,0.086
                         C78.754,44.977,78.743,44.933,78.743,44.933
                         z">
                 </path>
            </svg>
        );
    }
}

class PauseIcons extends React.Component {
    render() {
        const { onClick } = this.props;

        return (
            <svg className="icon" viewBox="0 0 20 20" onClick={onClick}>
                <polygon className="pause" points="4 0, 6 0, 6 20, 4 20"/>
                <polygon className="pause" points="13 0, 15 0, 15 20, 13 20"/>
            </svg>
        );
    }
}

export default class ControlIcons extends React.Component {
    render() {
        const { type, onClick } = this.props;

        switch(type) {
            case 'replay':
                return <ReplayIcons onClick={onClick} />;
            case 'pause':
                return <PauseIcons onClick={onClick} />;
            default:
                return <PlayIcons onClick={onClick} />;
        }
    }
}
