# Audio Playback Fix Notes

The audio playback path required three adjustments to eliminate the end-of-track echo, the loud screech, and the ten second media player hang:

1. **Coherent audio DMA writes.** Every USB frame copy now cleans the data cache for the updated buffer region so the SAI DMA always reads the fresh samples that the CPU just produced. This prevents the five second screech that occurred when stale cache lines reached the codec.

2. **Draining playback gracefully.** When the host closes the stream we wait briefly for the DMA engine to finish the queued samples before stopping the codec and we reset the buffer state. This keeps the media player from hanging on completion while ensuring the next playback starts from silence.

3. **Zeroing consumed buffers.** Each DMA half-transfer callback clears the region that was just rendered (and any underrun span) so no previously played audio can loop when the host stops sending data.

These targeted fixes match the structure of commit `803ba31` while preserving the behaviour improvements needed for reliable playback.
