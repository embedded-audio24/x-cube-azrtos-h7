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
#define USBD_AUDIO_STOP_DRAIN_MAX_MS   200U
#define USBD_AUDIO_STOP_DRAIN_MIN_MS   10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TX_QUEUE ux_app_MsgQueue;
extern UX_DEVICE_CLASS_AUDIO20_CONTROL audio_control[];

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

static HAL_StatusTypeDef USBD_AUDIO_SAIClockConfigure(uint32_t sample_rate,
                                                     uint64_t periph_clock,
                                                     uint32_t clock_source);
static uint32_t USBD_AUDIO_SAIClockFraction(uint32_t sample_rate,
                                            uint32_t multiplier,
                                            uint32_t pll2m,
                                            uint32_t pll2p,
                                            uint32_t pll2n);

static VOID USBD_AUDIO_FeedbackReset(VOID);
static VOID USBD_AUDIO_FeedbackStart(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                     ULONG sample_rate);
static VOID USBD_AUDIO_FeedbackUpdate(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                      ULONG queued_bytes);
static VOID USBD_AUDIO_FeedbackEncode(ULONG feedback_value,
                                      UINT high_speed,
                                      UCHAR encoded_feedback[4]);
static ULONG USBD_AUDIO_StopDrainBudget(ULONG pending_bytes);

static TX_SEMAPHORE USBD_AUDIO_SpaceSemaphore;
static UINT USBD_AUDIO_SpaceSemaphoreReady;
#if !defined(UX_DEVICE_STANDALONE)
static UINT USBD_AUDIO_StopPending;
static TX_SEMAPHORE USBD_AUDIO_StopSemaphore;
static UINT USBD_AUDIO_StopSemaphoreReady;
#endif

static ULONG USBD_AUDIO_FeedbackNominal;
static ULONG USBD_AUDIO_FeedbackMin;
static ULONG USBD_AUDIO_FeedbackMax;
static LONG  USBD_AUDIO_FeedbackIntegral;
static LONG  USBD_AUDIO_FeedbackIntegralLimit;
static LONG  USBD_AUDIO_FeedbackPGain;
static LONG  USBD_AUDIO_FeedbackIGain;
static ULONG USBD_AUDIO_FeedbackShift;
static ULONG USBD_AUDIO_FeedbackBytesPerSample;
static UINT  USBD_AUDIO_FeedbackPrimed;
static ULONG USBD_AUDIO_PlaybackBytesPerSecond;
#if !defined(UX_DEVICE_STANDALONE)
static ULONG USBD_AUDIO_StopWaitBudgetMs;
#endif

HAL_StatusTypeDef MX_SAI1_ClockConfig(SAI_HandleTypeDef *hsai,
                                      uint32_t SampleRate)
{
  UX_PARAMETER_NOT_USED(hsai);

  return USBD_AUDIO_SAIClockConfigure(SampleRate,
                                      RCC_PERIPHCLK_SAI1,
                                      RCC_SAI1CLKSOURCE_PLL2);
}

#if defined(RCC_PERIPHCLK_SAI4A)
HAL_StatusTypeDef MX_SAI4_ClockConfig(SAI_HandleTypeDef *hsai,
                                      uint32_t SampleRate)
{
  UX_PARAMETER_NOT_USED(hsai);

  return USBD_AUDIO_SAIClockConfigure(SampleRate,
                                      RCC_PERIPHCLK_SAI4A,
                                      RCC_SAI4ACLKSOURCE_PLL2);
}
#endif

static HAL_StatusTypeDef USBD_AUDIO_SAIClockConfigure(uint32_t sample_rate,
                                                     uint64_t periph_clock,
                                                     uint32_t clock_source)
{
  RCC_PeriphCLKInitTypeDef clk_config;
  HAL_StatusTypeDef status;
  uint32_t pll2_fraction = 0U;

  ux_utility_memory_set(&clk_config, 0, sizeof(clk_config));

  clk_config.PeriphClockSelection = periph_clock;
  clk_config.PLL2.PLL2M = 25U;
  clk_config.PLL2.PLL2N = 344U;
  clk_config.PLL2.PLL2P = 7U;
  clk_config.PLL2.PLL2Q = 2U;
  clk_config.PLL2.PLL2R = 2U;
  clk_config.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  clk_config.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;

  if ((sample_rate == AUDIO_FREQUENCY_48K) ||
      (sample_rate == AUDIO_FREQUENCY_96K) ||
      (sample_rate == AUDIO_FREQUENCY_192K) ||
      (sample_rate == AUDIO_FREQUENCY_32K) ||
      (sample_rate == AUDIO_FREQUENCY_16K) ||
      (sample_rate == AUDIO_FREQUENCY_8K))
  {
    pll2_fraction = USBD_AUDIO_SAIClockFraction(sample_rate,
                                                1024U,
                                                clk_config.PLL2.PLL2M,
                                                clk_config.PLL2.PLL2P,
                                                clk_config.PLL2.PLL2N);
  }
  else if ((sample_rate == AUDIO_FREQUENCY_44K) ||
           (sample_rate == AUDIO_FREQUENCY_22K) ||
           (sample_rate == AUDIO_FREQUENCY_11K) ||
           (sample_rate == AUDIO_FREQUENCY_88K) ||
           (sample_rate == AUDIO_FREQUENCY_176K))
  {
    clk_config.PLL2.PLL2P = 38U;
    clk_config.PLL2.PLL2N = 429U;
    pll2_fraction = USBD_AUDIO_SAIClockFraction(sample_rate,
                                                256U,
                                                clk_config.PLL2.PLL2M,
                                                clk_config.PLL2.PLL2P,
                                                clk_config.PLL2.PLL2N);
  }

  clk_config.PLL2.PLL2FRACN = pll2_fraction;

  if (periph_clock == RCC_PERIPHCLK_SAI1)
  {
    clk_config.Sai1ClockSelection = clock_source;
  }
#if defined(RCC_PERIPHCLK_SAI4A)
  else if (periph_clock == RCC_PERIPHCLK_SAI4A)
  {
    clk_config.Sai4AClockSelection = clock_source;
  }
#endif
  else
  {
    return HAL_ERROR;
  }

  status = HAL_RCCEx_PeriphCLKConfig(&clk_config);

  return status;
}

static uint32_t USBD_AUDIO_SAIClockFraction(uint32_t sample_rate,
                                            uint32_t multiplier,
                                            uint32_t pll2m,
                                            uint32_t pll2p,
                                            uint32_t pll2n)
{
  uint64_t numerator;
  uint64_t denominator;
  uint64_t integer_component;

  numerator = (uint64_t)sample_rate * (uint64_t)multiplier;
  numerator *= (uint64_t)pll2p * (uint64_t)pll2m;
  denominator = (uint64_t)HSE_VALUE;

  integer_component = numerator / denominator;
  if (integer_component != (uint64_t)pll2n)
  {
    return 0U;
  }

  numerator -= integer_component * denominator;

  numerator = (numerator * 8192ULL) + (denominator / 2ULL);
  numerator /= denominator;

  if (numerator > 8191ULL)
  {
    numerator = 8191ULL;
  }

  return (uint32_t)numerator;
}

static VOID USBD_AUDIO_FeedbackEncode(ULONG feedback_value,
                                      UINT high_speed,
                                      UCHAR encoded_feedback[4])
{
  encoded_feedback[0] = (UCHAR)(feedback_value & 0xFFU);
  encoded_feedback[1] = (UCHAR)((feedback_value >> 8) & 0xFFU);
  encoded_feedback[2] = (UCHAR)((feedback_value >> 16) & 0xFFU);

  if (high_speed != UX_FALSE)
  {
    encoded_feedback[3] = (UCHAR)((feedback_value >> 24) & 0xFFU);
  }
  else
  {
    encoded_feedback[3] = 0U;
  }
}

static VOID USBD_AUDIO_FeedbackReset(VOID)
{
  USBD_AUDIO_FeedbackNominal = 0U;
  USBD_AUDIO_FeedbackMin = 0U;
  USBD_AUDIO_FeedbackMax = 0U;
  USBD_AUDIO_FeedbackIntegral = 0;
  USBD_AUDIO_FeedbackIntegralLimit = 0;
  USBD_AUDIO_FeedbackPGain = 0;
  USBD_AUDIO_FeedbackIGain = 0;
  USBD_AUDIO_FeedbackShift = 0U;
  USBD_AUDIO_FeedbackBytesPerSample = 0U;
  USBD_AUDIO_FeedbackPrimed = UX_FALSE;
  USBD_AUDIO_PlaybackBytesPerSecond = 0U;
#if !defined(UX_DEVICE_STANDALONE)
  USBD_AUDIO_StopWaitBudgetMs = 0U;
#endif
}

static VOID USBD_AUDIO_FeedbackStart(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                     ULONG sample_rate)
{
  UINT high_speed;
  ULONG frame_rate;
  ULONG shift;
  ULONG bytes_per_sample;
  ULONG nominal;
  LONG integral_limit;

  if (audio_play_stream == UX_NULL)
  {
    USBD_AUDIO_FeedbackReset();

    return;
  }

  if (audio_play_stream->ux_device_class_audio_stream_feedback == UX_NULL)
  {
    USBD_AUDIO_FeedbackReset();

    return;
  }

  if (_ux_system_slave == UX_NULL)
  {
    USBD_AUDIO_FeedbackReset();

    return;
  }

  high_speed = (_ux_system_slave->ux_system_slave_speed == UX_HIGH_SPEED_DEVICE) ? UX_TRUE : UX_FALSE;
  frame_rate = (high_speed != UX_FALSE) ? 8000U : 1000U;
  shift = (high_speed != UX_FALSE) ? 16U : 14U;

  bytes_per_sample = (ULONG)USBD_AUDIO_PLAY_CHANNEL_COUNT * (ULONG)USBD_AUDIO_PLAY_RES_BYTE;
  if (bytes_per_sample == 0U)
  {
    bytes_per_sample = 1U;
  }

  if (sample_rate == 0U)
  {
    sample_rate = USBD_AUDIO_PLAY_DEFAULT_FREQ;
  }

  nominal = (ULONG)(((uint64_t)sample_rate << shift) / frame_rate);
  if (nominal == 0U)
  {
    nominal = 1U;
  }

  {
    uint64_t bytes_per_second;

    bytes_per_second = (uint64_t)sample_rate * (uint64_t)bytes_per_sample;
    if (bytes_per_second > (uint64_t)UINT32_MAX)
    {
      bytes_per_second = (uint64_t)UINT32_MAX;
    }

    USBD_AUDIO_PlaybackBytesPerSecond = (ULONG)bytes_per_second;
  }

  USBD_AUDIO_FeedbackShift = shift;
  USBD_AUDIO_FeedbackNominal = nominal;
  USBD_AUDIO_FeedbackBytesPerSample = bytes_per_sample;
  USBD_AUDIO_FeedbackPGain = (LONG)(1L << (shift - 7U));
  USBD_AUDIO_FeedbackIGain = (LONG)(1L << (shift - 11U));

  integral_limit = (LONG)(nominal / 16U);
  if (integral_limit <= 0)
  {
    integral_limit = (LONG)(1L << (shift - 2U));
  }
  USBD_AUDIO_FeedbackIntegralLimit = integral_limit;
  USBD_AUDIO_FeedbackIntegral = 0;

  USBD_AUDIO_FeedbackMin = nominal - (nominal / 16U);
  if (USBD_AUDIO_FeedbackMin == 0U)
  {
    USBD_AUDIO_FeedbackMin = 1U;
  }
  USBD_AUDIO_FeedbackMax = nominal + (nominal / 16U);
  if (USBD_AUDIO_FeedbackMax <= USBD_AUDIO_FeedbackMin)
  {
    USBD_AUDIO_FeedbackMax = USBD_AUDIO_FeedbackMin + 1U;
  }

  USBD_AUDIO_FeedbackPrimed = UX_TRUE;

  {
    UCHAR encoded_feedback[4];

    USBD_AUDIO_FeedbackEncode(nominal, high_speed, encoded_feedback);
    ux_device_class_audio_feedback_set(audio_play_stream, encoded_feedback);
  }
}

static VOID USBD_AUDIO_FeedbackUpdate(UX_DEVICE_CLASS_AUDIO_STREAM *audio_play_stream,
                                      ULONG queued_bytes)
{
  LONG queue_error_bytes;
  LONG queue_error_samples;
  LONG proportional;
  LONG integral;
  LONG feedback_value;
  UCHAR encoded_feedback[4];
  UINT high_speed;

  if ((USBD_AUDIO_FeedbackPrimed == UX_FALSE) ||
      (USBD_AUDIO_FeedbackBytesPerSample == 0U) ||
      (audio_play_stream == UX_NULL) ||
      (audio_play_stream->ux_device_class_audio_stream_feedback == UX_NULL))
  {
    return;
  }

  queue_error_bytes = (LONG)(AUDIO_TOTAL_BUF_SIZE / 2U) - (LONG)queued_bytes;
  queue_error_samples = queue_error_bytes / (LONG)USBD_AUDIO_FeedbackBytesPerSample;

  proportional = queue_error_samples * USBD_AUDIO_FeedbackPGain;
  integral = USBD_AUDIO_FeedbackIntegral + (queue_error_samples * USBD_AUDIO_FeedbackIGain);

  if (integral > USBD_AUDIO_FeedbackIntegralLimit)
  {
    integral = USBD_AUDIO_FeedbackIntegralLimit;
  }
  else if (integral < -USBD_AUDIO_FeedbackIntegralLimit)
  {
    integral = -USBD_AUDIO_FeedbackIntegralLimit;
  }

  USBD_AUDIO_FeedbackIntegral = integral;

  feedback_value = (LONG)USBD_AUDIO_FeedbackNominal + proportional + integral;
  if (feedback_value < (LONG)USBD_AUDIO_FeedbackMin)
  {
    feedback_value = (LONG)USBD_AUDIO_FeedbackMin;
  }
  else if (feedback_value > (LONG)USBD_AUDIO_FeedbackMax)
  {
    feedback_value = (LONG)USBD_AUDIO_FeedbackMax;
  }

  high_speed = (USBD_AUDIO_FeedbackShift == 16U) ? UX_TRUE : UX_FALSE;

  USBD_AUDIO_FeedbackEncode((ULONG)feedback_value, high_speed, encoded_feedback);
  ux_device_class_audio_feedback_set(audio_play_stream, encoded_feedback);
}

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

    if (timeout_ms == 0U)
    {
      timeout_ms = USBD_AUDIO_STOP_DRAIN_MIN_MS;
    }

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

  if (USBD_AUDIO_StopPending == UX_FALSE)
  {
    USBD_AUDIO_StopWaitBudgetMs = 0U;
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

  USBD_AUDIO_FeedbackReset();

#if !defined(UX_DEVICE_STANDALONE)
  USBD_AUDIO_StopPending = UX_FALSE;
  USBD_AUDIO_StopWaitBudgetMs = 0U;
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

static ULONG USBD_AUDIO_StopDrainBudget(ULONG pending_bytes)
{
  ULONG bytes_per_second = USBD_AUDIO_PlaybackBytesPerSecond;
  ULONG budget_ms;

  if (pending_bytes == 0U)
  {
    return 0U;
  }

  if (bytes_per_second == 0U)
  {
    budget_ms = USBD_AUDIO_STOP_DRAIN_MAX_MS;
  }
  else
  {
    uint64_t estimate_ms;

    estimate_ms = ((uint64_t)pending_bytes * 1000ULL) / (uint64_t)bytes_per_second;
    if (estimate_ms > (uint64_t)UINT32_MAX)
    {
      budget_ms = UINT32_MAX;
    }
    else
    {
      budget_ms = (ULONG)estimate_ms;
    }
  }

  if (budget_ms < USBD_AUDIO_STOP_DRAIN_MIN_MS)
  {
    budget_ms = USBD_AUDIO_STOP_DRAIN_MIN_MS;
  }
  if (budget_ms > USBD_AUDIO_STOP_DRAIN_MAX_MS)
  {
    budget_ms = USBD_AUDIO_STOP_DRAIN_MAX_MS;
  }

#if !defined(UX_DEVICE_STANDALONE)
  USBD_AUDIO_StopWaitBudgetMs = budget_ms;
#endif

  return budget_ms;
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
    ULONG pending_bytes;
    ULONG drain_budget;

    USBD_AUDIO_FeedbackPrimed = UX_FALSE;
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

    pending_bytes = USBD_AUDIO_BufferReserve(0U);
    drain_budget = USBD_AUDIO_StopDrainBudget(pending_bytes);

    if (pending_bytes != 0U)
    {
      USBD_AUDIO_BufferSilencePending();
    }

#if defined(UX_DEVICE_STANDALONE)
    if (drain_budget != 0U)
    {
      USBD_AUDIO_WaitForPlaybackDrain(drain_budget);
    }

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

    USBD_AUDIO_StopWaitForCompletion(drain_budget);
#endif

    return;
  }

  /* Reset local audio buffer state before starting a new playback stream. */
#if !defined(UX_DEVICE_STANDALONE)
  if (USBD_AUDIO_StopPending != UX_FALSE)
  {
    USBD_AUDIO_StopWaitForCompletion(USBD_AUDIO_StopWaitBudgetMs);
  }

  USBD_AUDIO_BufferReset();
#else
  USBD_AUDIO_BufferReset();
#endif

  USBD_AUDIO_FeedbackStart(audio_play_stream,
                           audio_control[0].ux_device_class_audio20_control_sampling_frequency);

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
  ULONG queued_bytes = 0U;
  
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

  if (frame_length == 0U)
  {
    queued_bytes = USBD_AUDIO_BufferReserve(0U);
  }

  USBD_AUDIO_FeedbackUpdate(audio_play_stream, queued_bytes);

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
        ULONG drain_budget;

        pending_bytes = USBD_AUDIO_BufferReserve(0U);
        drain_budget = USBD_AUDIO_StopDrainBudget(pending_bytes);

        if (pending_bytes != 0U)
        {
          USBD_AUDIO_BufferSilencePending();
          if (drain_budget != 0U)
          {
            USBD_AUDIO_WaitForPlaybackDrain(drain_budget);
          }
        }
        else if (drain_budget != 0U)
        {
          USBD_AUDIO_WaitForPlaybackDrain(drain_budget);
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
