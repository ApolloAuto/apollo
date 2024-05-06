import { action, computed, observable } from 'mobx';
import 'styles/playback-controls';

export default class Playback {
  FPS = 10; // frames per sec

  msPerFrame = 100;

  recordId = null;

  mapId = null;

    // real frame number starts from 1
    @observable numFrames = 0;

    @observable requestedFrame = 0;

    @observable retrievedFrame = 0;

    @observable loadingMarker = 0;

    @observable isPlaying = false;

    @observable isSeeking = true;

    @observable seekingFrame = 1;

    setMapId(mapId) {
      this.mapId = mapId;
    }

    setRecordId(recordId) {
      this.recordId = recordId;
    }

    setNumFrames(numFrames) {
      this.numFrames = parseInt(numFrames);
    }

    setPlayRate(rate) {
      if (typeof rate === 'number' && rate > 0) {
        const defaultSpeed = (1 / this.FPS * 1000);
        this.msPerFrame = defaultSpeed / rate;
      }
    }

    initialized() {
      return this.numFrames && this.recordId !== null && this.mapId !== null;
    }

    hasNext() {
      return this.initialized() && this.requestedFrame < this.numFrames;
    }

    @action next() {
      this.requestedFrame++;
      return this.requestedFrame;
    }

    @computed get currentFrame() {
      return this.retrievedFrame;
    }

    @computed get replayComplete() {
      return this.seekingFrame > this.numFrames;
    }

    @action setPlayAction(status) {
      this.isPlaying = status;
    }

    @action seekFrame(frame) {
      if (frame > 0 && frame <= this.numFrames) {
        this.seekingFrame = frame;
        this.requestedFrame = frame - 1;
        this.isSeeking = true;
      }
    }

    @action resetFrame() {
      this.requestedFrame = 0;
      this.retrievedFrame = 0;
      this.seekingFrame = 1;
    }

    @action shouldProcessFrame(world) {
      if (world && world.sequenceNum) {
        if (this.seekingFrame === world.sequenceNum && (this.isPlaying || this.isSeeking)) {
          this.retrievedFrame = world.sequenceNum;
          this.isSeeking = false;
          this.seekingFrame++;
          return true;
        }
      }

      return false;
    }

    @action setLoadingMarker(frameId) {
      this.loadingMarker = frameId;
    }
}
