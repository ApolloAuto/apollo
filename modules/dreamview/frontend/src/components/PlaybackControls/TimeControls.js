import React from "react";
import Slider from 'react-rangeslider';
import styled from 'styled-components';

export default class TimeControls extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            frame: -1,
            loadingProcess: '#2D3B50'
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
        if (nextProps.loadingMarker <= nextProps.numFrames) {
            const loadingPercent = (nextProps.loadingMarker / nextProps.numFrames) * 100;
            const unloadingPercent = 100 - loadingPercent;
            const backgroundColor = `linear-gradient(90deg, #212c3d ${loadingPercent.toFixed()}%,` +
                ` #2d3b50 ${loadingPercent.toFixed()}%, #2d3b50 ${unloadingPercent.toFixed()}%)`;
            this.setState({
                loadingProcess: backgroundColor
            });
        }
    }

    render() {
        const { numFrames, currentFrame, fps } = this.props;

        const totalTime = this.getTimeFromFrame(fps, numFrames);
        const currentTime = this.getTimeFromFrame(fps, currentFrame);

        const StyledSlider = styled(Slider)`
            background: ${this.state.loadingProcess}
        `;
        return (
            <div className="time-controls">
                <StyledSlider
                    tooltip={false}
                    min={1}
                    max={numFrames}
                    value={this.state.frame}
                    onChange={this.handleSliderChange}
                    onChangeComplete={this.handleSliderChangeComplete}
                />
                <div className="time-display">
                    {`${currentTime} / ${totalTime} s`}
                </div>
            </div>
        );
    }
}