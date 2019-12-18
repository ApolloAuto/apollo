import { observable, action } from "mobx";

export default class TeleopStore {
    @observable audioEnabled = false;
    @observable micEnabled = false;
    @observable videoEnabled = false;
    @observable modemInfo = observable.map();

    @action toggleAudio() {
        this.audioEnabled = !this.audioEnabled;
    }

    @action toggleMic() {
        this.micEnabled = !this.micEnabled;
    }

    @action toggleVideo() {
        this.videoEnabled = !this.videoEnabled;
    }

    @action update(status) {
        if (status.audio_starting) {
            this.audioEnabled = true;
        } else if (status.audio_stopping) {
            this.audioEnabled = false;
        } else {
            this.audioEnabled = status.audio;
        }

        if (status.mic_starting) {
            this.micEnabled = true;
        } else if (status.mic_stopping) {
            this.micEnabled = false;
        } else {
            this.micEnabled = status.mic;
        }

        if (status.video_starting) {
            this.videoEnabled = true;
        } else if (status.video_stopping) {
            this.videoEnabled = false;
        } else {
            this.videoEnabled = status.video;
        }

        for (const name in status.modems) {
            this.modemInfo.set(name, status.modems[name]);
        }
    }
}
