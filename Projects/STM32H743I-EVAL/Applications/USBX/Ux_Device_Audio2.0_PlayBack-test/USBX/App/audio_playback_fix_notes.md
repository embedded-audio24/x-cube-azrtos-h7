# Audio Playback Fix Notes

The audio playback path required three adjustments to eliminate the end-of-track echo, the loud screech, and the ten second media player hang:

1. **Coherent audio DMA writes.** Every USB frame copy now cleans the data cache for the updated buffer region so the SAI DMA always reads the fresh samples that the CPU just produced. This prevents the five second screech that occurred when stale cache lines reached the codec.

2. **Draining playback gracefully.** When the host closes the stream we mute the codec and hand the drain work to the playback thread so the USB control request returns immediately. The thread polls the buffered byte count directly, waits briefly for the DMA engine to finish the queued samples, lets the mute settle for a couple of milliseconds, and finally stops the SAI before resetting the buffer state while signalling completion back to the control side. Muting first removes the post-playback click while the asynchronous drain keeps the media player responsive and still guarantees the next playback starts from silence.

3. **Zeroing consumed buffers.** Each DMA half-transfer callback clears only the region that was just rendered (and any underrun span) so no previously played audio can loop when the host stops sending data while leaving in-flight USB writes untouched.

These targeted fixes match the structure of commit `803ba31` while preserving the behaviour improvements needed for reliable playback.
