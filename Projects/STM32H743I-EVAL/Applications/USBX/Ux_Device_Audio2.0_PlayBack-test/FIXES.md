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

## 8. Remaining work
- Verify the stop-drain cap on actual hardware and adjust `USBD_AUDIO_STOP_DRAIN_MAX_MS` if the codec needs a longer mute window.
- Collect USB analyzer traces to confirm the asynchronous feedback endpoint converges across every supported sample rate.
