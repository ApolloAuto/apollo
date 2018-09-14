class Utterance {
    utterance = window.speechSynthesis ? new SpeechSynthesisUtterance() : null;

    isSpeaking() {
        return this.utterance ? window.speechSynthesis.speaking : false;
    }

    speakOnce(text) {
        if (!this.utterance) {
            return;
        }

        this.stopSpeakingRepeatedly();
        this.utterance.text = text;
        window.speechSynthesis.speak(this.utterance);
    }

    speakRepeatedly(text) {
        if (!this.utterance) {
            return;
        }

        if (this.utterance.text !== text) {
            this.utterance.text = text;
            if (this.utterance.text) {
                // if speaking, don't interrupt
                if (!this.isSpeaking()) {
                    window.speechSynthesis.speak(this.utterance);
                }

                // repeat this message until a new one is given
                this.utterance.onend = () => {
                    window.speechSynthesis.speak(this.utterance);
                };
            } else {
                this.stopSpeakingRepeatedly();
            }
        }
    }

    stopSpeakingRepeatedly() {
        if (!this.utterance) {
            return;
        }

        this.utterance.onend = null;
    }

    cancelAllInQueue() {
        if (this.utterance) {
            window.speechSynthesis.cancel();
        }
    }

    getCurrentText() {
        return this.utterance ? this.utterance.text : null;
    }
}

const UTTERANCE = new Utterance();
export default UTTERANCE;
