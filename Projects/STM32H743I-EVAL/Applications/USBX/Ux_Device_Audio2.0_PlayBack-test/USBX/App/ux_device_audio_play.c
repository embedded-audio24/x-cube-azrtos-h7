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

static VOID USBD_AUDIO_BufferZero(ULONG start_index, ULONG length)
{
  ULONG remaining = length;

  if ((length == 0U) || (start_index >= AUDIO_TOTAL_BUF_SIZE))
  {
    return;
  }

  while (remaining != 0U)
  {
    ULONG segment_length = AUDIO_TOTAL_BUF_SIZE - start_index;

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

static VOID USBD_AUDIO_BufferReset(VOID)
{
  BufferCtl.rd_enable = 0U;
  BufferCtl.rd_ptr = 0U;
  BufferCtl.wr_ptr = 0U;
  BufferCtl.fptr = 0U;
  BufferCtl.state = PLAY_BUFFER_OFFSET_UNKNOWN;
  ux_utility_memory_set(BufferCtl.buff, 0, AUDIO_TOTAL_BUF_SIZE);
  USBD_AUDIO_CleanCache(BufferCtl.buff, AUDIO_TOTAL_BUF_SIZE);
}

static VOID USBD_AUDIO_PlaybackAdvance(ULONG bytes)
{
  ULONG consumed_bytes;
  ULONG underrun_bytes;
  ULONG start_index;

  if (bytes == 0U)
  {
    return;
  }

  if (bytes > AUDIO_TOTAL_BUF_SIZE)
  {
    bytes = AUDIO_TOTAL_BUF_SIZE;
  }

  start_index = BufferCtl.rd_ptr;

  if (BufferCtl.fptr >= bytes)
  {
    consumed_bytes = bytes;
    BufferCtl.fptr -= bytes;
  }
  else
  {
    consumed_bytes = BufferCtl.fptr;
    BufferCtl.fptr = 0U;
  }

  underrun_bytes = bytes - consumed_bytes;

  BufferCtl.rd_ptr += bytes;
  if (BufferCtl.rd_ptr >= AUDIO_TOTAL_BUF_SIZE)
  {
    BufferCtl.rd_ptr -= AUDIO_TOTAL_BUF_SIZE;
  }

  if (BufferCtl.fptr == 0U)
  {
    BufferCtl.rd_enable = 0U;
  }

  if (consumed_bytes != 0U)
  {
    /* Clear only the samples that the DMA actually rendered so the next stream
       begins from silence without touching regions the host may still fill. */
    USBD_AUDIO_BufferZero(start_index, consumed_bytes);
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
}

static VOID USBD_AUDIO_WaitForPlaybackDrain(ULONG timeout_ms)
{
  if ((timeout_ms == 0U) || (BufferCtl.rd_enable == 0U))
  {
    return;
  }

  ULONG timeout_ticks = MS_TO_TICK(timeout_ms);
  ULONG sleep_ticks = MS_TO_TICK(1U);
  ULONG start_tick = tx_time_get();

  if ((timeout_ms != 0U) && (timeout_ticks == 0U))
  {
    timeout_ticks = 1U;
  }

  if (sleep_ticks == 0U)
  {
    sleep_ticks = 1U;
  }

  while ((BufferCtl.fptr != 0U) && (BufferCtl.rd_enable != 0U))
  {
    if (timeout_ms != 0U)
    {
      ULONG elapsed = tx_time_get() - start_tick;

      if (elapsed >= timeout_ticks)
      {
        break;
      }
    }

    tx_thread_sleep(sleep_ticks);
  }
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

    if (endpoint != UX_NULL)
    {
      _ux_device_stack_transfer_all_request_abort(endpoint, UX_TRANSFER_STATUS_ABORT);
    }

    USBD_AUDIO_WaitForPlaybackDrain(1000U);

    BSP_AUDIO_OUT_Stop(0);

    USBD_AUDIO_BufferReset();

    return;
  }

  USBD_AUDIO_BufferReset();
#if defined(UX_DEVICE_STANDALONE)
  /* Make sure the standalone read task restarts immediately. */
  audio_play_stream->ux_device_class_audio_stream_task_state = UX_DEVICE_CLASS_AUDIO_STREAM_RW_START;
#endif

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

  ux_device_class_audio_read_frame_get(audio_play_stream, &frame_buffer, &frame_length);

  if ((frame_length != 0U) && (frame_length <= AUDIO_TOTAL_BUF_SIZE))
  {
    if ((AUDIO_TOTAL_BUF_SIZE - BufferCtl.fptr) >= frame_length)
    {
      ULONG remaining_length = frame_length;
      UCHAR *current_frame_ptr = frame_buffer;
      ULONG write_index = BufferCtl.wr_ptr;

      while (remaining_length != 0U)
      {
        ULONG space_until_wrap = AUDIO_TOTAL_BUF_SIZE - write_index;
        ULONG segment_length = (remaining_length < space_until_wrap) ? remaining_length : space_until_wrap;

        ux_utility_memory_copy(&BufferCtl.buff[write_index], current_frame_ptr, segment_length);
        USBD_AUDIO_CleanCache(&BufferCtl.buff[write_index], segment_length);

        current_frame_ptr += segment_length;
        write_index += segment_length;

        if (write_index == AUDIO_TOTAL_BUF_SIZE)
        {
          write_index = 0U;
        }

        remaining_length -= segment_length;
      }

      BufferCtl.wr_ptr = write_index;

      if (BufferCtl.fptr >= AUDIO_TOTAL_BUF_SIZE - frame_length)
      {
        BufferCtl.fptr = AUDIO_TOTAL_BUF_SIZE;
      }
      else
      {
        BufferCtl.fptr += frame_length;
      }

      if ((BufferCtl.rd_enable == 0U) &&
          (BufferCtl.fptr >= (AUDIO_TOTAL_BUF_SIZE / 2U)))
      {
        /* Match the original behaviour: start playback only after at least
           half the ring is primed to keep the DMA ahead of the host writes. */
        BufferCtl.rd_enable = 1U;
      }

      if ((BufferCtl.state == PLAY_BUFFER_OFFSET_UNKNOWN) && (BufferCtl.rd_enable != 0U))
      {
        BufferCtl.state = PLAY_BUFFER_OFFSET_NONE;

        if (tx_queue_send(&ux_app_MsgQueue, &BufferCtl.state, TX_NO_WAIT) != TX_SUCCESS)
        {
          Error_Handler();
        }
      }
    }
  }

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

  while (1)
  {
    if (tx_queue_receive(&ux_app_MsgQueue, &BufferCtl.state, TX_WAIT_FOREVER)!= TX_SUCCESS)
    {
      Error_Handler();
    }

    switch(BufferCtl.state)
    {

      case PLAY_BUFFER_OFFSET_NONE:

        USBD_AUDIO_CleanCache(BufferCtl.buff, AUDIO_TOTAL_BUF_SIZE);
        BSP_AUDIO_OUT_Play(0, (uint8_t*)&BufferCtl.buff[0], AUDIO_TOTAL_BUF_SIZE);

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
