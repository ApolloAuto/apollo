import React from "react";
import { inject, observer } from "mobx-react";

import Controls from "components/DashCamPlayer/Controls";

@inject("store") @observer
export default class DashCamPlayer extends React.Component {
    constructor (props) {
        super(props);

        this.state = {
            url: '',
            canPlay: false,
            hasStarted: false
        };

        this.loadVideo = this.loadVideo.bind(this);
        this.onCanplay = this.onCanplay.bind(this);
        this.setVideoPlayTime = this.setVideoPlayTime.bind(this);
        this.closeVideo = this.closeVideo.bind(this);
    }

    loadVideo(fileURL) {
        if (fileURL && fileURL.length !== 0) {
            this.videoElement.src = fileURL;
            this.videoElement.load();
        }
        this.setState({
            url: fileURL,
            canPlay: false,
            start: false
        });
    }

    onCanplay() {
        this.setState({
            canplay: true,
        });
    }

    videoTimeWithinRange() {
        const { video, worldTimestamp } = this.props.store;

        const startTime = video.startTime;
        const endTime   = startTime + this.videoElement.duration;

        return startTime <= worldTimestamp && endTime >= worldTimestamp;
    }

    setVideoPlayTime() {
        if (this.videoTimeWithinRange()) {
            const playTime =
                (this.props.store.worldTimestamp - this.props.store.video.startTime);
            this.setState({
               start: true
            });

            console.log("set video play time to ", playTime);
            this.videoElement.pause();
            this.videoElement.currentTime = playTime;
            this.videoElement.play();
        }
    }

    closeVideo() {
        this.props.store.video.resetVideo();
        delete(this.videoElement);
    }

    componentWillReceiveProps(nextProps) {
        const newPathFound = nextProps.store.video.path !== this.state.url;
        if (newPathFound) {
            this.loadVideo(nextProps.store.video.path);
        }
    }

    componentWillUpdate(nextProps, nextState) {
        const isReadyToPlay = nextState.canplay &&
                              nextProps.store.worldTimestamp !== 0;
        if (isReadyToPlay && !nextState.start) {
            this.setVideoPlayTime();;
        }
    }

    componentDidMount() {
        const { video } = this.props.store;
        this.loadVideo(video.path);
    }

    render() {
        const { startTime } = this.props.store.video;
        const showSyncup = (startTime !== 0 && this.videoTimeWithinRange());

        return (
           <div className="dashcam-player">
                    <Controls
                        showSyncup={showSyncup}
                        onSyncup={this.setVideoPlayTime}
                        onClose={this.closeVideo} />
                    <video controls
                           ref = {(input) => {
                                this.videoElement = input;
                           }}
                           onCanPlay={this.onCanplay} >
                           Your browser does not support the video type.
                    </video>
            </div>
        );
    }
}