import {action, computed, observable} from 'mobx';

export default class Playback {

    DEFAULT_MS_PER_FRAME = 100;
    msPerFrame = this.DEFAULT_MS_PER_FRAME;
    jobId = null;
    mapId = null;

    @observable numFrames = null;
    @observable currentFrame = null;

    setMapId(mapId) {
        this.mapId = mapId;
    }

    setJobId(jobId) {
        this.jobId = jobId;
    }

    setPlayRate(rate) {
       this.msPerFrame = this.DEFAULT_MS_PER_FRAME / rate;
    }

    @action setNumFrames(numFrames) {
        if (numFrames > 0) {
            this.numFrames = numFrames;
            this.currentFrame = 0;
        }
    }

    @action resetCurrentFrame() {
        this.currentFrame = 0;
    }

    @action updateCurrentFrameByPercentage(percentage) {
        this.currentFrame = Math.floor(this.numFrames * percentage / 100.0);
    }

    @computed get percentage() {
        return this.currentFrame/this.numFrames * 100;
    }

    @computed get totalTimeSec() {
        const numTimeIntervals = this.numFrames - 1;
        const totalTime = numTimeIntervals * this.DEFAULT_MS_PER_FRAME / 1000;
        return totalTime.toFixed(2);
    }

    @computed get currentTimeSec() {
        const numTimeIntervals = Math.max(0, this.currentFrame - 1);
        const currentTime = numTimeIntervals * this.DEFAULT_MS_PER_FRAME / 1000;
        return currentTime.toFixed(2);
    }

    @computed get replayComplete() {
        return this.currentFrame >= this.numFrames;
    }

    initialized() {
        return this.numFrames !== null && this.numFrames !== 0 &&
               this.jobId !== null && this.mapId !== null;
    }

    hasNext() {
        return this.initialized() && this.currentFrame < this.numFrames;
    }

    @action next() {
        this.currentFrame++;
        return this.currentFrame;
    }
}