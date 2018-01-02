import React from "react";
import Slider from 'react-rangeslider';

export default class TimeControls extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            frame: -1,
        };

        this.updatingSeekingTime = false;

        this.handleSliderChange = this.handleSliderChange.bind(this);
        this.handleSliderChangeComplete = this.handleSliderChangeComplete.bind(this);
    }

    getTimeFromFrame(fps, frame) {
        const numIntervals = Math.max(0, frame - 1);
        return (numIntervals / fps).toFixed(1);
    }

    handleSliderChange(frame) {
        this.setState({frame: frame});
        this.updatingSeekingTime = true;
    }

    handleSliderChangeComplete() {
        this.props.handleFrameSeek(this.state.frame);
        this.updatingSeekingTime = false;
    }

    componentWillReceiveProps(nextProps) {
        if (!this.updatingSeekingTime && !nextProps.isSeeking) {
            this.setState({frame: nextProps.currentFrame});
        }
    }

    render() {
        const { numFrames, currentFrame, fps,
                isSeeking, handleFrameSeek} = this.props;

        const totalTime = this.getTimeFromFrame(fps, numFrames);
        const currentTime = this.getTimeFromFrame(fps, currentFrame);

        return (
            <div className="time-controls">
                <Slider tooltip={false} min={1} max={numFrames}
                        value={this.state.frame}
                        onChange={this.handleSliderChange}
                        onChangeComplete={this.handleSliderChangeComplete}/>
                <div className="time-display">
                    {`${currentTime} / ${totalTime} s`}
                </div>
            </div>
        );
    }
}