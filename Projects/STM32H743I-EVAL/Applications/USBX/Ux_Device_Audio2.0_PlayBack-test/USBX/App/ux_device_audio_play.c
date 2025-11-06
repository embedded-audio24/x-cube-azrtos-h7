/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_audio_play.c
  * @author  MCD Application Team
  * @brief   USBX Device Video applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_audio_play.h"
#include "ux_device_stack.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TX_QUEUE ux_app_MsgQueue;

/* Set BufferCtl start address to "0x24040000" */
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = 0x24040000
#elif defined ( __CC_ARM ) || defined( __ARMCC_VERSION ) /* ARM Compiler 5/6 */
__attribute__((section(".AudioStreamBufferSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".AudioStreamBufferSection")))
#endif
/* Double BUFFER for Output Audio stream */
__ALIGN_BEGIN AUDIO_OUT_BufferTypeDef  BufferCtl __ALIGN_END;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if defined(SCB_CCR_DC_Msk)
static VOID USBD_AUDIO_CleanCache(VOID *address, ULONG size)
{
  uintptr_t aligned_address;
  uintptr_t aligned_size;
  uintptr_t end_address;

  if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U)
  {
    return;
  }

  aligned_address = ((uintptr_t)address) & ~(uintptr_t)0x1FU;
  end_address = (uintptr_t)address + (uintptr_t)size;
  aligned_size = end_address - aligned_address;
  aligned_size = (aligned_size + 31U) & ~(uintptr_t)0x1FU;

  SCB_CleanDCache_by_Addr((uint32_t *)aligned_address, (int32_t)aligned_size);
  __DSB();
  __ISB();
}
#else
static VOID USBD_AUDIO_CleanCache(VOID *address, ULONG size)
{
  UX_PARAMETER_NOT_USED(address);
  UX_PARAMETER_NOT_USED(size);
}
#endif

static inline uint32_t USBD_AUDIO_InterruptDisable(VOID)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  return primask;
}

static inline VOID USBD_AUDIO_InterruptRestore(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static TX_SEMAPHORE USBD_AUDIO_SpaceSemaphore;
static UINT USBD_AUDIO_SpaceSemaphoreReady;
#if !defined(UX_DEVICE_STANDALONE)
static UINT USBD_AUDIO_StopPending;
static TX_SEMAPHORE USBD_AUDIO_StopSemaphore;
static UINT USBD_AUDIO_StopSemaphoreReady;
#endif

static ULONG USBD_AUDIO_MillisecondsToTicks(ULONG milliseconds)
{
  ULONG ticks;

  ticks = (milliseconds * (ULONG)TX_TIMER_TICKS_PER_SECOND) + 999U;
  ticks /= 1000U;

  if ((ticks == 0U) && (milliseconds != 0U))
  {
    ticks = 1U;
  }

  return ticks;
}

#if !defined(UX_DEVICE_STANDALONE)
static VOID USBD_AUDIO_StopSemaphorePrepare(VOID)
{
  if (USBD_AUDIO_StopSemaphoreReady == UX_FALSE)
  {
    if (tx_semaphore_create(&USBD_AUDIO_StopSemaphore, "audio_stop", 0U) == TX_SUCCESS)
    {
      USBD_AUDIO_StopSemaphoreReady = UX_TRUE;
    }
  }

  if (USBD_AUDIO_StopSemaphoreReady != UX_FALSE)
  {
    while (tx_semaphore_get(&USBD_AUDIO_StopSemaphore, TX_NO_WAIT) == TX_SUCCESS)
    {
      /* Drain stale stop completions before waiting on a new one. */
    }
  }
}

static VOID USBD_AUDIO_StopWaitForCompletion(ULONG timeout_ms)
{
  if ((USBD_AUDIO_StopSemaphoreReady != UX_FALSE) &&
      (USBD_AUDIO_StopPending != UX_FALSE))
  {
    ULONG wait_ticks;

    wait_ticks = USBD_AUDIO_MillisecondsToTicks(timeout_ms);
    if (wait_ticks == 0U)
    {
      wait_ticks = 1U;
    }

    while (USBD_AUDIO_StopPending != UX_FALSE)
    {
      if (tx_semaphore_get(&USBD_AUDIO_StopSemaphore, wait_ticks) != TX_SUCCESS)
      {
        break;
      }
    }
  }
}
#endif

static VOID USBD_AUDIO_SpaceSemaphoreEnsureReady(VOID)
{
#if !defined(UX_DEVICE_STANDALONE)
  if (USBD_AUDIO_SpaceSemaphoreReady == UX_FALSE)
  {
    if (tx_semaphore_create(&USBD_AUDIO_SpaceSemaphore, "audio_space", 0U) == TX_SUCCESS)
    {
      USBD_AUDIO_SpaceSemaphoreReady = UX_TRUE;
    }
  }

  if (USBD_AUDIO_SpaceSemaphoreReady != UX_FALSE)
  {
    while (tx_semaphore_get(&USBD_AUDIO_SpaceSemaphore, TX_NO_WAIT) == TX_SUCCESS)
    {
      /* Drain stale space completions before waiting on a new one. */
    }

    tx_semaphore_ceiling_put(&USBD_AUDIO_SpaceSemaphore, 1U);
  }
#else
  /* No semaphore needed when running in standalone mode. */
#endif
}

static ULONG USBD_AUDIO_BufferReserve(ULONG length)
{
  ULONG used_bytes;
  uint32_t primask;

  if (length == 0U)
  {
    primask = USBD_AUDIO_InterruptDisable();
    used_bytes = BufferCtl.fptr;
    USBD_AUDIO_InterruptRestore(primask);

    return used_bytes;
  }

  if (length > AUDIO_TOTAL_BUF_SIZE)
  {
    length = AUDIO_TOTAL_BUF_SIZE;
  }

#if defined(UX_DEVICE_STANDALONE)
  primask = USBD_AUDIO_InterruptDisable();
  used_bytes = BufferCtl.fptr;

  if (used_bytes > AUDIO_TOTAL_BUF_SIZE)
  {
    used_bytes = AUDIO_TOTAL_BUF_SIZE;
    BufferCtl.fptr = AUDIO_TOTAL_BUF_SIZE;
  }

  if ((AUDIO_TOTAL_BUF_SIZE - used_bytes) >= length)
  {
    USBD_AUDIO_InterruptRestore(primask);

    return length;
  }

  USBD_AUDIO_InterruptRestore(primask);

  return AUDIO_TOTAL_BUF_SIZE + 1U;
#else
  USBD_AUDIO_SpaceSemaphoreEnsureReady();

  while (1)
  {
    primask = USBD_AUDIO_InterruptDisable();
    used_bytes = BufferCtl.fptr;

    if (used_bytes > AUDIO_TOTAL_BUF_SIZE)
    {
      used_bytes = AUDIO_TOTAL_BUF_SIZE;
      BufferCtl.fptr = AUDIO_TOTAL_BUF_SIZE;
    }

    if ((AUDIO_TOTAL_BUF_SIZE - used_bytes) >= length)
    {
      USBD_AUDIO_InterruptRestore(primask);

      return length;
    }

    USBD_AUDIO_InterruptRestore(primask);
    if (USBD_AUDIO_SpaceSemaphoreReady != UX_FALSE)
    {
      tx_semaphore_get(&USBD_AUDIO_SpaceSemaphore, TX_WAIT_FOREVER);
    }
    else
    {
      tx_thread_relinquish();
    }
  }
#endif
}

static ULONG USBD_AUDIO_BufferCommit(ULONG length)
{
  ULONG used_bytes;
  uint32_t primask;

  if (length == 0U)
  {
    primask = USBD_AUDIO_InterruptDisable();
    used_bytes = BufferCtl.fptr;
    USBD_AUDIO_InterruptRestore(primask);

    return used_bytes;
  }

  primask = USBD_AUDIO_InterruptDisable();
  used_bytes = BufferCtl.fptr + length;

  if (used_bytes > AUDIO_TOTAL_BUF_SIZE)
  {
    used_bytes = AUDIO_TOTAL_BUF_SIZE;
  }

  BufferCtl.fptr = used_bytes;
  USBD_AUDIO_InterruptRestore(primask);

  return used_bytes;
}

static VOID USBD_AUDIO_BufferZero(ULONG start_index, ULONG length)
{
  ULONG remaining = length;

  if ((length == 0U) || (start_index >= AUDIO_TOTAL_BUF_SIZE))
  {
    return;
  }

  while (remaining != 0U)
  {
    ULONG segment_length;

    segment_length = AUDIO_TOTAL_BUF_SIZE - start_index;
    if (segment_length > remaining)
    {
      segment_length = remaining;
    }

    ux_utility_memory_set(&BufferCtl.buff[start_index], 0, segment_length);
    USBD_AUDIO_CleanCache(&BufferCtl.buff[start_index], segment_length);

    start_index += segment_length;
    if (start_index >= AUDIO_TOTAL_BUF_SIZE)
    {
      start_index = 0U;
    }

    remaining -= segment_length;
  }
}

static VOID USBD_AUDIO_BufferSilencePending(VOID)
{
  ULONG read_index;
  ULONG pending_bytes;
  uint32_t primask;

  primask = USBD_AUDIO_InterruptDisable();
  read_index = BufferCtl.rd_ptr;
  pending_bytes = BufferCtl.fptr;
  USBD_AUDIO_InterruptRestore(primask);

  if (pending_bytes != 0U)
  {
    USBD_AUDIO_BufferZero(read_index, pending_bytes);
  }
}

static VOID USBD_AUDIO_PlaybackAdvance(ULONG bytes)
{
  uint32_t primask;
  ULONG start_index;
  ULONG consumed_bytes;
  ULONG underrun_bytes = 0U;
  ULONG zero_consumed_start = 0U;
  ULONG zero_consumed_length = 0U;

  if (bytes == 0U)
  {
    return;
  }

  if (bytes > AUDIO_TOTAL_BUF_SIZE)
  {
    bytes = AUDIO_TOTAL_BUF_SIZE;
  }

  primask = USBD_AUDIO_InterruptDisable();

  start_index = BufferCtl.rd_ptr;
  zero_consumed_start = start_index;
  consumed_bytes = bytes;
  if (BufferCtl.fptr >= consumed_bytes)
  {
    BufferCtl.fptr -= consumed_bytes;
  }
  else
  {
    consumed_bytes = BufferCtl.fptr;
    BufferCtl.fptr = 0U;
    underrun_bytes = bytes - consumed_bytes;
  }

  BufferCtl.rd_ptr = start_index + bytes;
  if (BufferCtl.rd_ptr >= AUDIO_TOTAL_BUF_SIZE)
  {
    BufferCtl.rd_ptr -= AUDIO_TOTAL_BUF_SIZE;
  }

  if (BufferCtl.fptr == 0U)
  {
    BufferCtl.rd_enable = 0U;
  }

  zero_consumed_length = consumed_bytes;

  USBD_AUDIO_InterruptRestore(primask);

  if (zero_consumed_length != 0U)
  {
    USBD_AUDIO_BufferZero(zero_consumed_start, zero_consumed_length);
  }

  if (underrun_bytes != 0U)
  {
    ULONG underrun_start = start_index + consumed_bytes;

    if (underrun_start >= AUDIO_TOTAL_BUF_SIZE)
    {
      underrun_start -= AUDIO_TOTAL_BUF_SIZE;
    }

    USBD_AUDIO_BufferZero(underrun_start, underrun_bytes);
  }

  if (USBD_AUDIO_SpaceSemaphoreReady != UX_FALSE)
  {
    tx_semaphore_ceiling_put(&USBD_AUDIO_SpaceSemaphore, 1U);
  }
}

static VOID USBD_AUDIO_BufferReset(VOID)
{
  BufferCtl.rd_enable = 0U;
  BufferCtl.rd_ptr = 0U;
  BufferCtl.wr_ptr = 0U;
  BufferCtl.fptr = 0U;
  BufferCtl.state = PLAY_BUFFER_OFFSET_UNKNOWN;
  ux_utility_memory_set(BufferCtl.buff, 0, AUDIO_TOTAL_BUF_SIZE);
  USBD_AUDIO_CleanCache(BufferCtl.buff, AUDIO_TOTAL_BUF_SIZE);

#if !defined(UX_DEVICE_STANDALONE)
  USBD_AUDIO_StopPending = UX_FALSE;
#endif

  if (USBD_AUDIO_SpaceSemaphoreReady != UX_FALSE)
  {
    while (tx_semaphore_get(&USBD_AUDIO_SpaceSemaphore, TX_NO_WAIT) == TX_SUCCESS)
    {
      /* Drain the semaphore to avoid stale wakeups after a reset. */
    }
    tx_semaphore_ceiling_put(&USBD_AUDIO_SpaceSemaphore, 1U);
  }
}

static VOID USBD_AUDIO_WaitForPlaybackDrain(ULONG timeout_ms)
{
#if defined(UX_DEVICE_STANDALONE)
  ULONG used_bytes;
  ULONG elapsed_ms;

  if (timeout_ms == 0U)
  {
    return;
  }

  /* Poll the buffered byte count directly—the playback flag may already be
     lowered even while the DMA drains the final samples. */
  used_bytes = USBD_AUDIO_BufferReserve(0U);
  for (elapsed_ms = 0U; (used_bytes != 0U) && (elapsed_ms < timeout_ms); elapsed_ms++)
  {
    ux_utility_delay_ms(1U);
    used_bytes = USBD_AUDIO_BufferReserve(0U);
  }
#else
  ULONG used_bytes;
  ULONG start_tick;
  ULONG timeout_ticks;
  ULONG sleep_ticks;

  if (timeout_ms == 0U)
  {
    return;
  }

  /* Poll the buffered byte count directly—the playback flag may already be
     lowered even while the DMA drains the final samples. */
  used_bytes = USBD_AUDIO_BufferReserve(0U);
  if (used_bytes == 0U)
  {
    return;
  }

  timeout_ticks = USBD_AUDIO_MillisecondsToTicks(timeout_ms);
  sleep_ticks = USBD_AUDIO_MillisecondsToTicks(1U);
  if (sleep_ticks == 0U)
  {
    sleep_ticks = 1U;
  }

  start_tick = tx_time_get();

  while (used_bytes != 0U)
  {
    ULONG current_tick;

    current_tick = tx_time_get();
    if ((current_tick - start_tick) >= timeout_ticks)
    {
      break;
    }

    tx_thread_sleep(sleep_ticks);

    used_bytes = USBD_AUDIO_BufferReserve(0U);
  }
#endif
}
/* USER CODE END 0 */

/**
  * @brief  USBD_AUDIO_PlaybackStreamChange
  *         This function is invoked to inform application that the
  *         alternate setting are changed.
  * @param  audio_play_stream: Pointer to audio playback class stream instance.
  * @param  alternate_setting: interface alternate setting.
  * @retval none
  */
VOID USBD_AUDIO_PlaybackStreamChange(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                     ULONG alternate_setting)
{
  /* USER CODE BEGIN USBD_AUDIO_PlaybackStreamChange */

  if (alternate_setting == 0U)
  {
    UX_SLAVE_ENDPOINT *endpoint = audio_play_stream->ux_device_class_audio_stream_endpoint;
    /* Stop host reception and local playback when the stream closes. */
#if defined(UX_DEVICE_STANDALONE)
    /* Mark the standalone read task as stopped so it flushes pending frames. */
    audio_play_stream->ux_device_class_audio_stream_task_state = UX_DEVICE_CLASS_AUDIO_STREAM_RW_STOP;
#endif

    if (endpoint != UX_NULL)
    {
      _ux_device_stack_transfer_all_request_abort(endpoint, UX_TRANSFER_STATUS_ABORT);
    }

    BSP_AUDIO_OUT_Mute(0);

#if defined(UX_DEVICE_STANDALONE)
    USBD_AUDIO_WaitForPlaybackDrain(1000U);

    BSP_AUDIO_OUT_Stop(0);

    /* Reset buffer state so stale samples are not replayed on next start. */
    USBD_AUDIO_BufferReset();
#else
    USBD_AUDIO_StopSemaphorePrepare();
    /* Let the playback thread finish draining without blocking the USB control path. */
    USBD_AUDIO_StopPending = UX_TRUE;

    BufferCtl.state = PLAY_BUFFER_OFFSET_STOP;
    if (tx_queue_send(&ux_app_MsgQueue, &BufferCtl.state, TX_NO_WAIT) != TX_SUCCESS)
    {
      Error_Handler();
    }

    USBD_AUDIO_StopWaitForCompletion(1000U);
#endif

    return;
  }

  /* Reset local audio buffer state before starting a new playback stream. */
#if !defined(UX_DEVICE_STANDALONE)
  if (USBD_AUDIO_StopPending != UX_FALSE)
  {
    USBD_AUDIO_StopWaitForCompletion(1000U);
  }

  USBD_AUDIO_BufferReset();
#else
  USBD_AUDIO_BufferReset();
#endif

#if defined(UX_DEVICE_STANDALONE)
  /* Make sure the standalone read task restarts immediately. */
  audio_play_stream->ux_device_class_audio_stream_task_state = UX_DEVICE_CLASS_AUDIO_STREAM_RW_START;
#endif

  /* Start reception (stream opened).  */
  ux_device_class_audio_reception_start(audio_play_stream);

  /* USER CODE END USBD_AUDIO_PlaybackStreamChange */

  return;
}

/**
  * @brief  USBD_AUDIO_PlaybackStreamFrameDone
  *         This function is invoked whenever a USB packet (audio frame) is received
  *         from the host.
  * @param  audio_play_stream: Pointer to audio playback class stream instance.
  * @param  length: transfer length.
  * @retval none
  */
VOID USBD_AUDIO_PlaybackStreamFrameDone(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                        ULONG length)
{
  /* USER CODE BEGIN USBD_AUDIO_PlaybackStreamFrameDone */

  UX_PARAMETER_NOT_USED(length);

  UCHAR *frame_buffer;
  ULONG frame_length;
  ULONG reserve_result;
  
  /* Get access to first audio input frame.  */
  ux_device_class_audio_read_frame_get(audio_play_stream, &frame_buffer, &frame_length);

#if !defined(UX_DEVICE_STANDALONE)
  if (USBD_AUDIO_StopPending != UX_FALSE)
  {
    ux_device_class_audio_read_frame_free(audio_play_stream);

    return;
  }
#endif

  if (frame_length != 0U)
  {
    ULONG remaining_chunk = frame_length;
    UCHAR *current_frame_ptr = frame_buffer;
    ULONG write_index = BufferCtl.wr_ptr;
    ULONG queued_bytes;

    reserve_result = USBD_AUDIO_BufferReserve(frame_length);

#if defined(UX_DEVICE_STANDALONE)
    if (reserve_result > AUDIO_TOTAL_BUF_SIZE)
    {
      /* Not enough room and cannot wait in standalone mode, drop the frame. */
      ux_device_class_audio_read_frame_free(audio_play_stream);

      return;
    }
#else
    UX_PARAMETER_NOT_USED(reserve_result);
#endif

    while (remaining_chunk != 0U)
    {
      ULONG space_until_wrap = AUDIO_TOTAL_BUF_SIZE - write_index;
      ULONG segment_length = (remaining_chunk < space_until_wrap) ? remaining_chunk : space_until_wrap;

      ux_utility_memory_copy(&BufferCtl.buff[write_index], current_frame_ptr, segment_length);
      current_frame_ptr += segment_length;

      if (segment_length != 0U)
      {
        USBD_AUDIO_CleanCache(&BufferCtl.buff[write_index], segment_length);
      }

      write_index += segment_length;

      if (write_index == AUDIO_TOTAL_BUF_SIZE)
      {
        write_index = 0U;
      }

      remaining_chunk -= segment_length;
    }

    BufferCtl.wr_ptr = write_index;

    queued_bytes = USBD_AUDIO_BufferCommit(frame_length);

    if (queued_bytes > AUDIO_TOTAL_BUF_SIZE)
    {
      queued_bytes = AUDIO_TOTAL_BUF_SIZE;
    }

    if ((BufferCtl.rd_enable == 0U) &&
        (queued_bytes >= (AUDIO_TOTAL_BUF_SIZE / 2U)))
    {
      BufferCtl.rd_enable = 1U;
    }

    if ((BufferCtl.state == PLAY_BUFFER_OFFSET_UNKNOWN) &&
        (BufferCtl.rd_enable != 0U)
#if !defined(UX_DEVICE_STANDALONE)
        && (USBD_AUDIO_StopPending == UX_FALSE)
#endif
        )
    {
      /* Start BSP play */
      BufferCtl.state = PLAY_BUFFER_OFFSET_NONE;

      /* Put a message queue  */
      if(tx_queue_send(&ux_app_MsgQueue, &BufferCtl.state, TX_NO_WAIT) != TX_SUCCESS)
      {
        Error_Handler();
      }
    }

  }

  /* Re-free the first audio input frame for transfer.  */
  ux_device_class_audio_read_frame_free(audio_play_stream);

  /* USER CODE END USBD_AUDIO_PlaybackStreamFrameDone */

  return;
}

/**
  * @brief  USBD_AUDIO_PlaybackStreamGetMaxFrameBufferNumber
  *         This function is invoked to Set audio playback stream max Frame buffer number.
  * @param  none
  * @retval max frame buffer number
  */
ULONG USBD_AUDIO_PlaybackStreamGetMaxFrameBufferNumber(VOID)
{
  ULONG max_frame_buffer_number = 0U;

  /* USER CODE BEGIN USBD_AUDIO_PlaybackStreamGetMaxFrameBufferNumber */

  max_frame_buffer_number = 3U;

  /* USER CODE END USBD_AUDIO_PlaybackStreamGetMaxFrameBufferNumber */

  return max_frame_buffer_number;
}

/**
  * @brief  USBD_AUDIO_PlaybackStreamGetMaxFrameBufferSize
  *         This function is invoked to Set audio playback stream max Frame buffer size.
  * @param  none
  * @retval max frame buffer size
  */
ULONG USBD_AUDIO_PlaybackStreamGetMaxFrameBufferSize(VOID)
{
  ULONG max_frame_buffer_size = 0U;

  /* USER CODE BEGIN USBD_AUDIO_PlaybackStreamGetMaxFrameBufferSize */

  max_frame_buffer_size = USBD_AUDIO_PLAY_EPOUT_HS_MPS;

  /* USER CODE END USBD_AUDIO_PlaybackStreamGetMaxFrameBufferSize */

  return max_frame_buffer_size;
}

/* USER CODE BEGIN 2 */
/**
  * @brief  Function implementing usbx_app_thread_entry.
  * @param arg: Not used
  * @retval None
  */
VOID usbx_audio_play_app_thread(ULONG arg)
{

  UX_PARAMETER_NOT_USED(arg);

  USBD_AUDIO_SpaceSemaphoreEnsureReady();

  if (USBD_AUDIO_SpaceSemaphoreReady == UX_FALSE)
  {
    Error_Handler();
  }

  while (1)
  {
    /* Wait for a hid device to be connected */
    if (tx_queue_receive(&ux_app_MsgQueue, &BufferCtl.state, TX_WAIT_FOREVER)!= TX_SUCCESS)
    {
      Error_Handler();
    }

    switch(BufferCtl.state)
    {

      case PLAY_BUFFER_OFFSET_STOP:

      {
        ULONG pending_bytes;
        UINT should_stop = UX_TRUE;

        pending_bytes = USBD_AUDIO_BufferReserve(0U);

        if (pending_bytes != 0U)
        {
          USBD_AUDIO_BufferSilencePending();
          USBD_AUDIO_WaitForPlaybackDrain(1000U);
        }
#if !defined(UX_DEVICE_STANDALONE)
        {
          ULONG settle_ticks;

          /* Let the mute propagate for a moment before stopping the SAI. */
          settle_ticks = USBD_AUDIO_MillisecondsToTicks(2U);
          if (settle_ticks == 0U)
          {
            settle_ticks = 1U;
          }

          tx_thread_sleep(settle_ticks);
        }
#endif
        {
          uint32_t audio_state = AUDIO_OUT_STATE_STOP;

          if (BSP_AUDIO_OUT_GetState(0, &audio_state) == BSP_ERROR_NONE)
          {
            should_stop = (audio_state == AUDIO_OUT_STATE_PLAYING) ? UX_TRUE : UX_FALSE;
          }

          if (should_stop == UX_TRUE)
          {
            (void)BSP_AUDIO_OUT_Stop(0);
          }
        }
        BSP_AUDIO_OUT_Mute(0);
        USBD_AUDIO_BufferReset();
#if !defined(UX_DEVICE_STANDALONE)
        if (USBD_AUDIO_StopSemaphoreReady != UX_FALSE)
        {
          tx_semaphore_ceiling_put(&USBD_AUDIO_StopSemaphore, 1U);
        }
#endif

        break;
      }

      case PLAY_BUFFER_OFFSET_NONE:

        /*DMA stream from output double buffer to codec in Circular mode launch*/
        USBD_AUDIO_CleanCache(BufferCtl.buff, AUDIO_TOTAL_BUF_SIZE);
        BSP_AUDIO_OUT_Play(0, (uint8_t*)&BufferCtl.buff[0], AUDIO_TOTAL_BUF_SIZE);
        BSP_AUDIO_OUT_UnMute(0);

        break;

    default:
      tx_thread_sleep(MS_TO_TICK(10));
      break;
    }
  }
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
VOID BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  UX_PARAMETER_NOT_USED(Instance);

  USBD_AUDIO_PlaybackAdvance(AUDIO_TOTAL_BUF_SIZE / 2U);
}

VOID BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  UX_PARAMETER_NOT_USED(Instance);

  USBD_AUDIO_PlaybackAdvance(AUDIO_TOTAL_BUF_SIZE / 2U);
}
/* USER CODE END 3 */
