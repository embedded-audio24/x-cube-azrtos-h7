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
    if (endpoint != UX_NULL)
    {
      ux_device_stack_transfer_all_request_abort(endpoint, UX_TRANSFER_STATUS_ABORT);
    }

    BSP_AUDIO_OUT_Stop(0);

    /* Reset buffer state so stale samples are not replayed on next start. */
    BufferCtl.rd_enable = 0U;
    BufferCtl.rd_ptr = 0U;
    BufferCtl.wr_ptr = 0U;
    BufferCtl.fptr = 0U;
    BufferCtl.state = PLAY_BUFFER_OFFSET_UNKNOWN;
    ux_utility_memory_set(BufferCtl.buff, 0, AUDIO_TOTAL_BUF_SIZE);

    return;
  }

  /* Reset local audio buffer state before starting a new playback stream. */
  BufferCtl.rd_enable = 0U;
  BufferCtl.rd_ptr = 0U;
  BufferCtl.wr_ptr = 0U;
  BufferCtl.fptr = 0U;
  ux_utility_memory_set(BufferCtl.buff, 0, AUDIO_TOTAL_BUF_SIZE);

  BufferCtl.state = PLAY_BUFFER_OFFSET_UNKNOWN;
  audio_play_stream->ux_device_class_audio_stream_task_state =
    UX_DEVICE_CLASS_AUDIO_STREAM_RW_START;

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

  UCHAR *frame_buffer;
  ULONG frame_length;

  /* Get access to first audio input frame.  */
  ux_device_class_audio_read_frame_get(audio_play_stream, &frame_buffer, &frame_length);

  if (frame_length != 0U)
  {
    ULONG next_wr_ptr;
    ULONG remaining_chunk = frame_length;
    ULONG remaining_copy = length;
    UCHAR *current_frame_ptr = frame_buffer;
    UINT buffer_filled = UX_FALSE;
    ULONG write_index = BufferCtl.wr_ptr;

    /* Calculate the write pointer position after storing the frame.  */
    next_wr_ptr = write_index + frame_length;

    while (remaining_chunk != 0U)
    {
      ULONG space_until_wrap = AUDIO_TOTAL_BUF_SIZE - write_index;
      ULONG segment_length = (remaining_chunk < space_until_wrap) ? remaining_chunk : space_until_wrap;
      ULONG segment_copy = (remaining_copy < segment_length) ? remaining_copy : segment_length;

      if (segment_copy != 0U)
      {
        ux_utility_memory_copy(&BufferCtl.buff[write_index], current_frame_ptr, segment_copy);
        current_frame_ptr += segment_copy;
        remaining_copy -= segment_copy;
      }

      if (segment_length > segment_copy)
      {
        ux_utility_memory_set(&BufferCtl.buff[write_index + segment_copy], 0,
                              segment_length - segment_copy);
      }

      write_index += segment_length;

      if (write_index == AUDIO_TOTAL_BUF_SIZE)
      {
        write_index = 0U;
        buffer_filled = UX_TRUE;
      }

      remaining_chunk -= segment_length;
    }

    BufferCtl.wr_ptr = write_index;

    if ((BufferCtl.rd_enable == 0U) &&
        (next_wr_ptr >= (AUDIO_TOTAL_BUF_SIZE / 2U)))
    {
      BufferCtl.rd_enable = 1U;
    }

    if ((BufferCtl.state == PLAY_BUFFER_OFFSET_UNKNOWN) && (buffer_filled == UX_TRUE))
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

  while (1)
  {
    /* Wait for a hid device to be connected */
    if (tx_queue_receive(&ux_app_MsgQueue, &BufferCtl.state, TX_WAIT_FOREVER)!= TX_SUCCESS)
    {
      Error_Handler();
    }

    switch(BufferCtl.state)
    {

      case PLAY_BUFFER_OFFSET_NONE:

        /*DMA stream from output double buffer to codec in Circular mode launch*/
        BSP_AUDIO_OUT_Play(0, (uint8_t*)&BufferCtl.buff[0], AUDIO_TOTAL_BUF_SIZE);

        break;

    default:
      tx_thread_sleep(MS_TO_TICK(10));
      break;
    }
  }
}
/* USER CODE END 2 */
