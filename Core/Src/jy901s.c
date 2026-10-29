/* STM32F4/FreeRTOS port of WITMOTION's STM32 NORMAL SDK. See SDK provenance. */
#include "jy901s.h"
#include "usart.h"
#include "cmsis_os.h"
#include "heading_pid.h"
#include "wit_c_sdk.h"
#include <math.h>
#include <string.h>

#define DMA_SIZE             128U
#define QUEUE_SIZE           512U
#define PAIR_MAX_AGE_MS       20U
#define STATIONARY_TIME_MS    1990U
#define READ_NONE             0xffffU

static uint8_t s_dma[DMA_SIZE]; /* Ordinary SRAM; DMA cannot access CCM RAM. */
static uint8_t s_bytes[QUEUE_SIZE];
static uint32_t s_times[QUEUE_SIZE];
static volatile uint32_t s_head, s_tail;
static volatile bool s_accept_rx, s_transport_fault;
static volatile JY901S_Diagnostic s_rx_reason;
static volatile uint32_t s_uart_error;
static uint16_t s_dma_position;
static JY901S_Feedback s_work, s_feedback;
static uint32_t s_parse_ms, s_started_ms, s_stationary_ms;
static uint32_t s_expected_read = READ_NONE;
static uint16_t s_read_values[4];
static bool s_read_seen, s_tx_failed, s_configured;
static uint8_t s_pair;
static bool s_have_pair, s_have_angle, s_have_gyro;
static float s_reference_yaw, s_stationary_yaw;

static void Publish(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  s_feedback = s_work;
  if (mask == 0U) __enable_irq();
}

void JY901S_GetFeedback(JY901S_Feedback *feedback)
{
  uint32_t mask;
  if (feedback == NULL) return;
  mask = __get_PRIMASK();
  __disable_irq();
  *feedback = s_feedback;
  if (mask == 0U) __enable_irq();
}

bool JY901S_IsFresh(const JY901S_Feedback *feedback, uint32_t now_ms)
{
  return feedback != NULL && feedback->status == JY901S_READY &&
    (uint32_t)(now_ms - feedback->last_angle_ms) <= JY901S_STALE_TIMEOUT_MS &&
    (uint32_t)(now_ms - feedback->last_gyro_ms) <= JY901S_STALE_TIMEOUT_MS &&
    (uint32_t)(now_ms - feedback->last_sample_ms) <= JY901S_STALE_TIMEOUT_MS;
}

/* IDLE, half-complete and complete all use this callback in circular mode.
 * Do not disable the DMA half-transfer interrupt: continuous streams need it. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  uint32_t received_ms;
  uint16_t end;
  if (uart != &huart3 || !s_accept_rx || size == 0U || size > DMA_SIZE) return;
  received_ms = HAL_GetTick();
  end = (uint16_t)(size % DMA_SIZE);
  while (s_dma_position != end)
  {
    uint32_t head = s_head;
    if ((uint32_t)(head - s_tail) >= QUEUE_SIZE)
    {
      s_rx_reason = JY901S_DIAG_QUEUE;
      s_transport_fault = true;
      return;
    }
    s_bytes[head % QUEUE_SIZE] = s_dma[s_dma_position];
    s_times[head % QUEUE_SIZE] = received_ms;
    __DMB();
    s_head = head + 1U;
    s_dma_position = (uint16_t)((s_dma_position + 1U) % DMA_SIZE);
  }
}

void JY901S_UartError(UART_HandleTypeDef *uart)
{
  if (uart == &huart3 && s_accept_rx) {
    s_uart_error = uart->ErrorCode;
    s_rx_reason = JY901S_DIAG_UART;
    s_transport_fault = true;
  }
}

static HAL_StatusTypeDef Fault(JY901S_Diagnostic reason)
{
  s_accept_rx = false;
  s_configured = false;
  (void)HAL_UART_AbortReceive(&huart3);
  if (s_work.status != JY901S_FAULT) ++s_work.error_count;
  s_work.status = JY901S_FAULT;
  s_work.fault_reason = s_transport_fault ? s_rx_reason : (s_tx_failed ? JY901S_DIAG_TX : reason);
  s_work.uart_error = s_uart_error;
  Publish();
  return HAL_ERROR;
}

static void ValidatePair(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t separation;
  if (!s_configured || s_pair != 3U) return;
  s_pair = 0U;
  separation = (s_work.last_angle_ms >= s_work.last_gyro_ms) ?
    s_work.last_angle_ms - s_work.last_gyro_ms : s_work.last_gyro_ms - s_work.last_angle_ms;
  /* Unsigned minimum also handles HAL tick wrap. */
  if (separation > 0x80000000U) separation = 0U - separation;
  if (separation > PAIR_MAX_AGE_MS ||
      (uint32_t)(now - s_work.last_angle_ms) > JY901S_STALE_TIMEOUT_MS ||
      (uint32_t)(now - s_work.last_gyro_ms) > JY901S_STALE_TIMEOUT_MS)
  {
    return;
  }

  /* Do not join stationary samples across a stream interruption, including
   * when Poll did not run during the gap but fresh frames have now arrived. */
  if (s_work.status == JY901S_CALIBRATING && s_have_pair &&
      (uint32_t)(s_parse_ms - s_work.last_sample_ms) > JY901S_STALE_TIMEOUT_MS)
    s_work.calibration_samples = 0U;
  s_work.last_sample_ms = s_parse_ms;
  s_have_pair = true;
  if (s_work.status == JY901S_CALIBRATING)
  {
    bool stationary = fabsf(s_work.roll_deg) <= 10.0f &&
      fabsf(s_work.pitch_deg) <= 10.0f;
    for (uint32_t axis = 0U; axis < 3U; ++axis)
      if (fabsf(s_work.gyro_dps[axis]) > 2.0f) stationary = false;
    if (s_work.calibration_samples != 0U &&
        fabsf(HeadingPID_WrapDegrees(s_work.raw_yaw_deg - s_stationary_yaw)) > 0.5f)
      stationary = false;
    if (!stationary) s_work.calibration_samples = 0U;
    else
    {
      if (s_work.calibration_samples == 0U)
      {
        s_stationary_ms = s_parse_ms;
        s_stationary_yaw = s_work.raw_yaw_deg;
      }
      if (s_work.calibration_samples < JY901S_CALIBRATION_SAMPLES)
        ++s_work.calibration_samples;
      if (s_work.calibration_samples >= JY901S_CALIBRATION_SAMPLES &&
          (uint32_t)(s_parse_ms - s_stationary_ms) >= STATIONARY_TIME_MS)
      {
        s_reference_yaw = s_work.raw_yaw_deg;
        s_work.status = JY901S_READY;
        s_work.fault_reason = JY901S_DIAG_NONE;
        s_work.uart_error = 0U;
      }
    }
  }
  if (s_work.status == JY901S_READY)
  {
    /* Use module attitude directly; never integrate it again on the MCU. */
    s_work.yaw_deg = HeadingPID_WrapDegrees(JY901S_YAW_SIGN *
      HeadingPID_WrapDegrees(s_work.raw_yaw_deg - s_reference_yaw));
    s_work.yaw_rate_dps = JY901S_YAW_SIGN * s_work.gyro_dps[2];
  }
  Publish();
}

/* Called synchronously by WitSerialDataIn in imuTask, never from an IRQ. */
static void RegistersUpdated(uint32_t reg, uint32_t count)
{
  if (reg == s_expected_read && count == 4U)
  {
    for (uint32_t i = 0U; i < 4U; ++i) s_read_values[i] = (uint16_t)sReg[reg + i];
    s_read_seen = true;
  }
  if (reg == GX && count == 3U)
  {
    for (uint32_t i = 0U; i < 3U; ++i)
      s_work.gyro_dps[i] = (float)sReg[GX + i] * (2000.0f / 32768.0f);
    s_work.last_gyro_ms = s_parse_ms;
    s_have_gyro = true;
    s_pair |= 1U;
    ValidatePair();
  }
  else if (reg == Roll && count == 3U)
  {
    s_work.roll_deg = (float)sReg[Roll] * (180.0f / 32768.0f);
    s_work.pitch_deg = (float)sReg[Pitch] * (180.0f / 32768.0f);
    s_work.raw_yaw_deg = (float)sReg[Yaw] * (180.0f / 32768.0f);
    s_work.last_angle_ms = s_parse_ms;
    s_have_angle = true;
    s_pair |= 2U;
    ValidatePair();
  }
}

static void Drain(void)
{
  /* Bounded work; overflowing the queue is a fault, never a partial recovery. */
  uint32_t available = s_head - s_tail;
  if (available > QUEUE_SIZE) { s_rx_reason=JY901S_DIAG_QUEUE; s_transport_fault = true; return; }
  while (available-- != 0U && !s_transport_fault)
  {
    uint32_t tail = s_tail;
    uint8_t byte;
    __DMB();
    byte = s_bytes[tail % QUEUE_SIZE];
    s_parse_ms = s_times[tail % QUEUE_SIZE];
    __DMB();
    s_tail = tail + 1U;
    WitSerialDataIn(byte);
  }
}

static void WaitMs(uint32_t ms)
{
  uint32_t start = HAL_GetTick();
  uint32_t ticks = (osKernelGetTickFreq() + 999U) / 1000U;
  do {
    Drain();
    (void)osDelay(ticks);
  } while ((uint32_t)(HAL_GetTick() - start) < ms && !s_transport_fault);
  Drain();
}

static void SensorSerialWrite(uint8_t *data, uint32_t length)
{
  if (HAL_UART_Transmit(&huart3, data, (uint16_t)length, 30U) != HAL_OK)
    s_tx_failed = true;
}

static HAL_StatusTypeDef StartTransport(uint32_t baud)
{
  s_accept_rx = false;
  (void)HAL_UART_AbortReceive(&huart3);
  huart3.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart3) != HAL_OK) return HAL_ERROR;
  s_head = s_tail = 0U;
  s_dma_position = 0U;
  s_transport_fault = s_tx_failed = false;
  s_rx_reason = JY901S_DIAG_NONE;
  s_uart_error = 0U;
  s_expected_read = READ_NONE;
  s_pair = 0U;
  (void)WitInit(WIT_PROTOCOL_NORMAL, 0x50U);
  (void)WitRegisterCallBack(RegistersUpdated);
  (void)WitSerialWriteRegister(SensorSerialWrite);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  s_accept_rx = true;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dma, DMA_SIZE) != HAL_OK)
  {
    s_accept_rx = false;
    return HAL_ERROR;
  }
  return HAL_OK;
}

static bool ReadRegistersOnce(uint32_t reg)
{
  uint32_t start;
  Drain();
  s_expected_read = reg;
  s_read_seen = false;
  if (WitReadReg(reg, 4U) != WIT_HAL_OK) return false;
  start = HAL_GetTick();
  while (!s_read_seen && !s_transport_fault && !s_tx_failed &&
         (uint32_t)(HAL_GetTick() - start) < 150U) WaitMs(1U);
  s_expected_read = READ_NONE;
  return s_read_seen && !s_transport_fault && !s_tx_failed;
}

static bool ReadRegisters(uint32_t reg)
{
  /* Retry the same address before scanning away from a working baud rate. */
  for (unsigned attempt=0; attempt<3U; ++attempt) {
    if (attempt!=0U) {
      if (s_transport_fault || s_tx_failed) {
        if (StartTransport(huart3.Init.BaudRate)!=HAL_OK) return false;
      }
      WaitMs(20U);
    }
    if (ReadRegistersOnce(reg)) return true;
  }
  return false;
}

static bool WriteRegister(uint32_t reg, uint16_t value)
{
  if (WitWriteReg(reg, value) != WIT_HAL_OK) return false;
  WaitMs(20U);
  return !s_tx_failed && !s_transport_fault;
}

HAL_StatusTypeDef JY901S_Init(void)
{
  static const uint32_t baud_rates[] = {115200U, 9600U, 57600U, 38400U,
                                       19200U, 230400U, 4800U};
  uint32_t old_errors = s_work.error_count;
  JY901S_Diagnostic old_reason = s_work.fault_reason;
  uint32_t old_uart_error = s_work.uart_error;
  uint32_t found_baud = 0U;
  uint16_t output = 0U, rate = 0U, algorithm;
  bool changed;
  s_configured = false;
  s_accept_rx = false;
  (void)HAL_UART_AbortReceive(&huart3);
  s_transport_fault = s_tx_failed = false;
  s_rx_reason = JY901S_DIAG_NONE;
  s_uart_error = 0U;
  s_have_pair = false;
  memset(&s_work, 0, sizeof(s_work));
  s_work.error_count = old_errors;
  s_work.fault_reason = old_reason;
  s_work.uart_error = old_uart_error;
  Publish();

  /* Covers the documented 1s power-on startup without blocking controlTask. */
  (void)osDelay(osKernelGetTickFreq());
  for (uint32_t i = 0U; i < sizeof(baud_rates) / sizeof(baud_rates[0]); ++i)
  {
    if (StartTransport(baud_rates[i]) == HAL_OK && ReadRegisters(RSW))
    {
      /* A real read response verifies RX and TX, including silent-output mode. */
      if (s_read_values[1] <= 0x0dU && s_read_values[2] >= 1U && s_read_values[2] <= 7U)
      {
        found_baud = baud_rates[i];
        output = s_read_values[0];
        rate = s_read_values[1];
        break;
      }
    }
  }
  if (found_baud == 0U) return Fault(JY901S_DIAG_PROBE);
  if (!ReadRegisters(AXIS6)) return Fault(JY901S_DIAG_CONFIG_READ);
  algorithm = s_read_values[0];
  changed = found_baud != JY901S_UART_BAUD || output != 0x000cU ||
            rate != 0x0009U || algorithm != 1U;
  if (changed)
  {
    if (!WriteRegister(KEY, KEY_UNLOCK)) return Fault(JY901S_DIAG_TX);
    WaitMs(200U);
    if (found_baud != JY901S_UART_BAUD)
    {
      if (!WriteRegister(BAUD, 6U) || StartTransport(JY901S_UART_BAUD) != HAL_OK)
        return Fault(JY901S_DIAG_TRANSPORT);
      if (!WriteRegister(KEY, KEY_UNLOCK)) return Fault(JY901S_DIAG_TX);
      WaitMs(200U);
    }
    if (!WriteRegister(RSW, 0x000cU) || !WriteRegister(RRATE, 9U) ||
        !WriteRegister(AXIS6, 1U)) return Fault(JY901S_DIAG_TX);
    /* Confirm writes before storing; do not wear module flash on each retry. */
    if (!ReadRegisters(RSW) || s_read_values[0] != 0x000cU ||
        s_read_values[1] != 9U || s_read_values[2] != 6U ||
        !ReadRegisters(AXIS6) || s_read_values[0] != 1U) return Fault(JY901S_DIAG_CONFIG_VERIFY);
    if (!WriteRegister(KEY, KEY_UNLOCK)) return Fault(JY901S_DIAG_TX);
    WaitMs(200U);
    if (!WriteRegister(SAVE, 0U)) return Fault(JY901S_DIAG_TX);
    WaitMs(100U);
  }
  if (!ReadRegisters(RSW) || s_read_values[0] != 0x000cU ||
      s_read_values[1] != 9U || s_read_values[2] != 6U ||
      !ReadRegisters(AXIS6) || s_read_values[0] != 1U) return Fault(JY901S_DIAG_CONFIG_VERIFY);
  /* Discard setup traffic and start stationary validation from new samples. */
  if (StartTransport(JY901S_UART_BAUD) != HAL_OK) return Fault(JY901S_DIAG_TRANSPORT);
  s_have_angle = s_have_gyro = false;
  s_work.calibration_samples = 0U;
  s_work.status = JY901S_CALIBRATING;
  s_started_ms = HAL_GetTick();
  s_configured = true;
  Publish();
  return HAL_OK;
}

void JY901S_Poll(void)
{
  uint32_t now, stream_timeout;
  if (!s_configured) return;
  Drain();
  now = HAL_GetTick();
  stream_timeout = s_work.status == JY901S_CALIBRATING ?
    JY901S_CALIBRATION_TIMEOUT_MS : JY901S_STALE_TIMEOUT_MS;
  /* Before READY, a short missing frame restarts stationary validation rather
   * than entering the full baud/configuration retry loop. Never arm on gaps. */
  if (s_work.status == JY901S_CALIBRATING && s_work.calibration_samples != 0U &&
      ((uint32_t)(now - s_work.last_angle_ms) > JY901S_STALE_TIMEOUT_MS ||
       (uint32_t)(now - s_work.last_gyro_ms) > JY901S_STALE_TIMEOUT_MS ||
       (uint32_t)(now - s_work.last_sample_ms) > JY901S_STALE_TIMEOUT_MS))
  {
    s_work.calibration_samples = 0U;
    Publish();
  }
  if (s_transport_fault) (void)Fault(s_rx_reason);
  else if ((uint32_t)(now-(s_have_angle ? s_work.last_angle_ms : s_started_ms)) >
           (s_have_angle ? stream_timeout : JY901S_FIRST_DATA_TIMEOUT_MS))
    (void)Fault(JY901S_DIAG_ANGLE_TIMEOUT);
  else if ((uint32_t)(now-(s_have_gyro ? s_work.last_gyro_ms : s_started_ms)) >
           (s_have_gyro ? stream_timeout : JY901S_FIRST_DATA_TIMEOUT_MS))
    (void)Fault(JY901S_DIAG_GYRO_TIMEOUT);
  else if ((uint32_t)(now-(s_have_pair ? s_work.last_sample_ms : s_started_ms)) >
           (s_have_pair ? stream_timeout : JY901S_FIRST_DATA_TIMEOUT_MS))
    (void)Fault(JY901S_DIAG_PAIR_TIMEOUT);
}
