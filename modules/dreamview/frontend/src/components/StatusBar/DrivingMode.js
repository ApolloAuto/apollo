import React from "react";
import classNames from "classnames";

export default class DrivingMode extends React.PureComponent {
    constructor(props) {
        super(props);
        this.utterance = new SpeechSynthesisUtterance();
    }

    componentWillUpdate() {
        window.speechSynthesis.cancel();
    }

    render() {
        const { drivingMode, isAutoMode } = this.props;

        this.utterance.text = `Entering to ${drivingMode} mode`;
        window.speechSynthesis.speak(this.utterance);

        return (
            <div className={classNames({
                        "driving-mode": true,
                        "auto-mode": isAutoMode,
                        "manual-mode": !isAutoMode,
                    })}>
                <span className="text">{drivingMode}</span>
            </div>
        );
    }
}
