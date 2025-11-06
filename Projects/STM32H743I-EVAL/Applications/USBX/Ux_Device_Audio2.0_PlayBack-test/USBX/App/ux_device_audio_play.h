/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_audio_play.h
  * @author  MCD Application Team
  * @brief   USBX Device audio plyaback applicative header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UX_DEVICE_AUDIO_PLAY_H__
#define __UX_DEVICE_AUDIO_PLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "ux_api.h"
#include "ux_device_class_audio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ux_device_class_audio20.h"
#include "ux_device_descriptors.h"
#include "stm32h743i_eval_audio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
VOID USBD_AUDIO_PlaybackStreamChange(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                     ULONG alternate_setting);
VOID USBD_AUDIO_PlaybackStreamFrameDone(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                        ULONG length);
ULONG USBD_AUDIO_PlaybackStreamGetMaxFrameBufferNumber(VOID);
ULONG USBD_AUDIO_PlaybackStreamGetMaxFrameBufferSize(VOID);
/* USER CODE BEGIN EFP */
VOID usbx_audio_play_app_thread(ULONG arg);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Size of audio buffer*/
#define AUDIO_TOTAL_BUF_SIZE   192*600

#define USBD_AUDIO_DEBUG_ENABLED        1U
#define USBD_AUDIO_DEBUG_LOG_SIZE       128U

typedef enum
{
  USBD_AUDIO_DEBUG_EVENT_STREAM_OPEN = 1U,     /* value0: alternate setting */
  USBD_AUDIO_DEBUG_EVENT_STREAM_CLOSE,         /* value0: queued bytes, value1: alternate setting */
  USBD_AUDIO_DEBUG_EVENT_BUFFER_RESET,         /* values unused */
  USBD_AUDIO_DEBUG_EVENT_FRAME_RECEIVED,       /* value0: queued bytes before copy, value1: frame length */
  USBD_AUDIO_DEBUG_EVENT_FRAME_DROPPED,        /* value0: queued bytes before drop, value1: frame length */
  USBD_AUDIO_DEBUG_EVENT_RESERVE_WAIT,         /* value0: queued bytes when wait started, value1: requested bytes */
  USBD_AUDIO_DEBUG_EVENT_RESERVE_GRANTED,      /* value0: queued bytes when granted, value1: reserved bytes */
  USBD_AUDIO_DEBUG_EVENT_FRAME_COMMIT,         /* value0: queued bytes after commit, value1: committed bytes */
  USBD_AUDIO_DEBUG_EVENT_PLAYBACK_ADVANCE,     /* value0: queued bytes after DMA advance, value1: consumed bytes */
  USBD_AUDIO_DEBUG_EVENT_PLAYBACK_START,       /* value0: queued bytes, value1: stage (1=prime,2=queue,3=DMA) */
  USBD_AUDIO_DEBUG_EVENT_SEMAPHORE_PUT         /* value0: queued bytes after release */
} USBD_AUDIO_DebugEvent;

typedef struct
{
  ULONG tick;
  ULONG event;
  ULONG value0;
  ULONG value1;
} USBD_AUDIO_DebugEntry;

typedef enum
{
  AUDIOPLAYER_STOP = 0,
  AUDIOPLAYER_START,
  AUDIOPLAYER_PLAY,
  AUDIOPLAYER_PAUSE,
  AUDIOPLAYER_EOF,
  AUDIOPLAYER_ERROR,
}AUDIOPLAYER_StateTypedef;

typedef struct
{
  uint32_t        volume;
  uint32_t        mute;
  uint32_t        freq;
  AUDIOPLAYER_StateTypedef state;
}AUDIO_ProcessTypdef ;

typedef enum {
  PLAY_BUFFER_OFFSET_UNKNOWN = 0,
  PLAY_BUFFER_OFFSET_NONE,
  PLAY_BUFFER_OFFSET_HALF,
  PLAY_BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

/* Audio buffer control structure */
typedef struct {
  uint8_t buff[AUDIO_TOTAL_BUF_SIZE];
  BUFFER_StateTypeDef state;   /* empty (none), half, full*/
  uint8_t rd_enable;
  uint32_t rd_ptr;
  uint32_t wr_ptr;
  uint32_t fptr;
}AUDIO_OUT_BufferTypeDef;
/* USER CODE END PD */

/* USER CODE BEGIN 2 */

VOID USBD_AUDIO_DebugLogReset(VOID);
VOID USBD_AUDIO_DebugLogSnapshot(USBD_AUDIO_DebugEntry *entries, ULONG max_entries, ULONG *out_count);
const USBD_AUDIO_DebugEntry *USBD_AUDIO_DebugLogSnapshotGet(ULONG *out_count);

/* USER CODE END 2 */

#ifdef __cplusplus
}
#endif
#endif  /* __UX_DEVICE_AUDIO_PLAY_H__ */
