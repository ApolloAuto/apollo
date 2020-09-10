This serves as the audio driver folder

## Microphone Configuration

* **microphone_model**: currently only support `RESPEAKER`.
* **chunk**: number of frames per buffer from hardware to memory each time.
* **record_seconds**: number of seconds each time.
* **channel_type**
  * May be ASR (Audio Speech Recognition), RAW (Raw Audio Data) or Playback.
* **sample_rate**
  * Number of frames that are recorded per second.
* **sample_width**
  * Number of bytes per sample (1, 2, 3, or 4).

For example, if there are 6 channels with 16000 Hz rate and 2 bytes width, then 4 second recording is
* Number of frames: 16000
* Number of "samples": 16000 * 6
  * I use the term "samples" here just to make it easy for understanding, which is seldom used in this context.
* Total size: 6 × 16,000 x 4 × 2 = 768,000 bytes.

You might see other metrics elsewhere as follows:

* **BIT DEPTH** same to sample_width except that the unit is bit.
* **BIT RATE** number of bits encoded per second (kbps or k) -- for compressed format like mp3.

## Output
  * Raw microphone data (cyber channel `apollo/sensor/microphone`).
