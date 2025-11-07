# Audio Playback Fix Log

This document captures the issues addressed during the USB Audio Class 2.0 playback bring-up and the code areas touched so you can file follow-up tickets or revisit the changes quickly.

## 1. Playback jitter and buffer starvation
- **Symptom**: Audio broke down in 10-second cycles because the playback task spun when the circular buffer ran dry.
- **Fix**: Added the `USBD_AUDIO_SpaceSemaphore` flow control so producers block until DMA catches up instead of hammering the buffer, and reset it with `USBD_AUDIO_BufferReset()` to avoid stale wakeups. See `USBX/App/ux_device_audio_play.c`.

## 2. Incorrect SAI clock source
- **Symptom**: The codec wandered off-pitch because SAI1/SAI4 stayed on the internal RC oscillator.
- **Fix**: Hooked `MX_SAI1_ClockConfig()` (and SAI4 when present) to retarget the bus clocks at PLL2 with the appropriate fractional settings for 44.1/48 kHz families. See `USBX/App/ux_device_audio_play.c`.

## 3. Host/device drift without asynchronous feedback
- **Symptom**: Long streams slowly distorted; the host never corrected for device-side clock drift.
- **Fix**: Enabled the optional feedback endpoint, registered the USBX feedback thread, and implemented `USBD_AUDIO_FeedbackStart()/Update()` to publish rate adjustments derived from the playback queue depth. See `USBX/App/ux_device_audio_play.c`, `USBX/App/ux_device_descriptors.c`, and `USBX/App/app_usbx_device.c`.

## 4. Build failures after enabling feedback support
- **Symptom**: The project stopped compiling because the new controller state was defined below its first use.
- **Fix**: Hoisted the feedback globals and semaphore declarations to the top of `ux_device_audio_play.c` and guarded them with the proper `UX_DEVICE_STANDALONE` checks.

## 5. Residual pops and 10-second stalls after track changes
- **Symptom**: Audio pops persisted and the UI hung ~10 seconds before replay after pressing Next or when a playlist ended.
- **Fix**: Corrected the playback error sign and tightened the PI integral clamp so feedback requests speed up when the buffer under-runs, and capped the stop-drain wait time with `USBD_AUDIO_StopDrainBudget()` so low-rate streams do not block the next start. See `USBX/App/ux_device_audio_play.c`.

## 6. Long stop handshakes after playlist changes
- **Symptom**: Even after bounding the drain itself, the host still paused ~10 seconds before starting the next track because the class request handler always waited a full second for the stop thread to finish.
- **Fix**: Track the computed drain budget in `USBD_AUDIO_StopDrainBudget()` and reuse it when `USBD_AUDIO_StopWaitForCompletion()` blocks the control path, guaranteeing the wait matches the actual buffer flush time instead of the previous fixed 1-second timeout. See `USBX/App/ux_device_audio_play.c`.

## 7. Stop completion watchdog still stalling
- **Symptom**: Even with the shorter drain window the host could still hang for ~10 seconds because the control handler waited on the stop semaphore until the USB control transfer itself timed out.
- **Fix**: Made `USBD_AUDIO_StopWaitForCompletion()` poll the semaphore in millisecond slices, track the elapsed budget, and forcibly mute/stop/reset the stream when the worker misses that deadline. The playback thread sees the `USBD_AUDIO_StopForced` flag and skips duplicate stop work so the next track can start immediately. See `USBX/App/ux_device_audio_play.c`.

## 8. Stop watchdog fallback did not clear follow-on starts
- **Symptom**: The 10-second pause resurfaced whenever the host started another track immediately after a forced stop because the pending stop never cleared fast enough, so the new open blocked until the control transfer timeout expired.
- **Fix**: Consolidated the watchdog logic into `USBD_AUDIO_StopForceComplete()` so both the control path and the playback worker can reset the buffer, post the semaphore, and flag the forced stop. Startup now calls the helper if the previous stop is still pending, guaranteeing the state machine unlocks before opening the next stream. See `USBX/App/ux_device_audio_play.c`.

## 9. Stop watchdog fallback guard
- **Symptom**: Builds without the stop semaphore or with a failed RTOS primitive allocation still waited the full USB control timeout because the watchdog path never ran.
- **Fix**: `USBD_AUDIO_StopWaitForCompletion()` now bails out immediately when the stop semaphore is unavailable and calls `USBD_AUDIO_StopForceComplete()` so the buffer, DMA, and state flags reset before the host times out. The helper also short-circuits if the stop has already finished to keep the cached wait budget consistent. See `USBX/App/ux_device_audio_play.c`.

## 10. Stop completion short-circuit for empty drains
- **Symptom**: Pressing Next or replay after a playlist ended still paused around 10 seconds because the stop handler always
  queued the worker path and waited out the drain budget even when no audio remained in the buffer.
- **Fix**: Cache a zero wait budget when the queue is already empty and force-complete the stop immediately instead of queuing
  the playback thread. The control path now returns as soon as the host closes the stream. See `USBX/App/ux_device_audio_play.c`.

## 11. Stop wait still stalling playlist transitions
- **Symptom**: After the watchdog changes the host still hesitated ~10 seconds when pressing Next or when playback finished because
  each control request waited through the full drain budget before the forced-stop path ran.
- **Fix**: Clamp `USBD_AUDIO_StopWaitForCompletion()` to a single short semaphore probe and fall back to `USBD_AUDIO_StopForceComplete()`
  immediately so the control transfer resumes without exhausting the budget. The worker still observes `USBD_AUDIO_StopForced`
  and skips redundant draining. See `USBX/App/ux_device_audio_play.c`.

## 12. Forced stop still blocked on HAL stop
- **Symptom**: After the one-shot wait change the UI pauses stretched to ~30 seconds because the forced-stop helper itself blocked inside `BSP_AUDIO_OUT_Stop()` while the DMA finished flushing.
- **Fix**: Let `USBD_AUDIO_StopForceComplete()` only mute and reset state, then let the playback thread perform the actual hardware stop with a new `skip_drain` path so it bypasses the long drain/poll loop. The forced flag still posts the stop semaphore immediately so control requests return without waiting for the worker. See `USBX/App/ux_device_audio_play.c`.

## 13. Remaining work
- Verify the stop-drain cap on actual hardware and adjust `USBD_AUDIO_STOP_DRAIN_MAX_MS` if the codec needs a longer mute window.
- Collect USB analyzer traces to confirm the asynchronous feedback endpoint converges across every supported sample rate.
