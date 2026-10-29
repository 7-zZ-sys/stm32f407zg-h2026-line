#include "Vofa.h"

#include "usart.h"
#include "jy901s.h"

#define VOFA_UART_TIMEOUT_MS  20U
#define VOFA_RX_BUFFER_SIZE   128U

/*
 * 两路串口的角色分工：
 *   USART1 = PA9/PA10，USB-TTL 接电脑，跑 VOFA 上位机（本地调试）
 *   USART2 = PD5/PD6，接 ESP8266，把同一套文本协议透传到 TCP（脱线调参）
 *
 * 两路用的是**同一套协议**：遥测下行是 Vofa_Printf 的文本行，指令上行是
 * V=/W=/KP=/FF= 这些文本命令。所以上位机侧不需要区分，VOFA+ 连串口还是
 * 连 TCP 都一样。
 *
 * 行为：
 *   - 遥测**两路都发**（镜像）。代价是每帧发两遍，115200 下多花约 9 ms；
 *     好处是 USB 和无线可以同时看。
 *   - 指令**两路都收**，谁后到谁生效。解析仍统一在 communicationTask 里
 *     通过 VofaPort_ReadEvent() 按来源取字节，独立解析，不在中断里做。
 *
 * 中断里只搬字节：Chassis_SetVelocity()/Chassis_SetHeading() 带临界区且会
 * 改写控制状态，绝不能在 ISR 里调用。
 */

#define VOFA_PORT_USB    0U  /* USART1 */
#define VOFA_PORT_WIFI   1U  /* USART2 */

typedef struct
{
  volatile uint8_t buffer[VOFA_RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint8_t reset;
  volatile uint8_t discard;
  volatile uint32_t overflows;
  volatile uint32_t errors;
  volatile uint8_t rearm_needed;
  uint8_t byte;
} VofaPortRx;

static VofaPortRx s_rx[VOFA_PORT_COUNT];
static uint8_t s_next_port;
static volatile uint32_t s_tx_errors[VOFA_PORT_COUNT];
static UART_HandleTypeDef *const s_uart[VOFA_PORT_COUNT] = {&huart1, &huart2};

/* 由中断里的 huart 反查是哪一路；不属于本模块的返回 -1。 */
static int32_t VofaPortIndexOf(UART_HandleTypeDef *huart)
{
  for (uint32_t index = 0U; index < VOFA_PORT_COUNT; ++index)
  {
    if (huart->Instance == s_uart[index]->Instance)
    {
      return (int32_t)index;
    }
  }
  return -1;
}

static void VofaPortArmRx(uint32_t index)
{
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(s_uart[index], &s_rx[index].byte, 1U);
  s_rx[index].rearm_needed = (status != HAL_OK && status != HAL_BUSY);
  if (s_rx[index].rearm_needed) ++s_rx[index].errors;
}

void VofaPort_Init(void)
{
  s_next_port = 0U;
  for (uint32_t index = 0U; index < VOFA_PORT_COUNT; ++index)
  {
    s_rx[index].head = 0U;
    s_rx[index].tail = 0U;
    s_rx[index].reset = 0U;
    s_rx[index].discard = 0U;
    s_rx[index].overflows = 0U;
    s_rx[index].errors = 0U;
    s_tx_errors[index] = 0U;
    VofaPortArmRx(index);
  }
}

static uint8_t VofaPortPop(uint32_t index, uint8_t *byte)
{
  VofaPortRx *rx = &s_rx[index];

  if (rx->head == rx->tail)
  {
    return 0U;
  }

  *byte = rx->buffer[rx->tail];
  rx->tail = (uint16_t)((rx->tail + 1U) % VOFA_RX_BUFFER_SIZE);
  return 1U;
}

uint8_t VofaPort_ReadEvent(uint8_t *port, uint8_t *byte, uint8_t *flags)
{
  if (port == NULL || byte == NULL || flags == NULL) return 0U;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t i = 0; i < VOFA_PORT_COUNT; ++i)
    if (s_rx[i].rearm_needed) VofaPortArmRx(i);
  for (uint32_t n = 0; n < VOFA_PORT_COUNT; ++n) {
    uint8_t index = (uint8_t)((s_next_port + n) % VOFA_PORT_COUNT);
    *flags = 0U;
    *byte = 0U;
    if (s_rx[index].reset != 0U) {
      *flags = s_rx[index].reset;
      s_rx[index].reset = 0U;
    } else if (VofaPortPop(index, byte) == 0U) continue;
    *port = index;
    s_next_port = (uint8_t)((index + 1U) % VOFA_PORT_COUNT);
    if (primask == 0U) __enable_irq();
    return 1U;
  }
  if (primask == 0U) __enable_irq();
  return 0U;
}

uint8_t VofaPort_ReadByte(uint8_t *byte)
{
  uint8_t port, flags;
  if (byte == NULL) return 0U;
  while (VofaPort_ReadEvent(&port, byte, &flags)) {
    if (flags == 0U) return 1U;
  }
  return 0U;
}

void VofaPort_DiscardPendingInput(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t i = 0; i < VOFA_PORT_COUNT; ++i) {
    s_rx[i].tail = s_rx[i].head;
    s_rx[i].reset |= VOFA_RX_EVENT_FLUSH;
  }
  if (primask == 0U) __enable_irq();
}

void VofaPort_GetStats(VofaPortStats *stats)
{
  if (stats == NULL) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t i = 0; i < VOFA_PORT_COUNT; ++i) {
    stats->rx_overflows[i] = s_rx[i].overflows;
    stats->rx_errors[i] = s_rx[i].errors;
    stats->tx_errors[i] = s_tx_errors[i];
  }
  if (primask == 0U) __enable_irq();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  int32_t index = VofaPortIndexOf(huart);

  if (index < 0)
  {
    return;
  }

  {
    VofaPortRx *rx = &s_rx[index];
    uint16_t next_head = (uint16_t)((rx->head + 1U) % VOFA_RX_BUFFER_SIZE);

    /* 溢出后整条指令作废，丢弃到行尾；绝不执行留下的数字前缀。 */
    if (rx->discard != 0U) {
      if (rx->byte == '\r' || rx->byte == '\n') {
        rx->discard = 0U;
        rx->buffer[rx->head] = rx->byte;
        rx->head = next_head;
      }
    } else if (next_head != rx->tail)
    {
      rx->buffer[rx->head] = rx->byte;
      rx->head = next_head;
    } else {
      rx->tail = rx->head;
      rx->reset = 1U;
      rx->discard = 1U;
      ++rx->overflows;
      if (rx->byte == '\r' || rx->byte == '\n') {
        rx->discard = 0U;
        rx->buffer[rx->head] = rx->byte;
        rx->head = next_head;
      }
    }
  }

  VofaPortArmRx((uint32_t)index);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3) { JY901S_UartError(huart); return; }
  int32_t index = VofaPortIndexOf(huart);

  if (index >= 0)
  {
    s_rx[index].tail = s_rx[index].head;
    s_rx[index].reset = 1U;
    s_rx[index].discard = 1U;
    ++s_rx[index].errors;
    VofaPortArmRx((uint32_t)index);
  }
}

void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data,
                           uint16_t length)
{
  (void)handle;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  /*
   * 两路都发。USART2 没接 ESP 时也不会阻塞——UART 发送不需要有接收方，
   * 移完寄存器就结束，只是这些字节没人收而已。
   */
  if (HAL_UART_Transmit(&huart1, data, length, VOFA_UART_TIMEOUT_MS) != HAL_OK)
    ++s_tx_errors[VOFA_PORT_USB];
  if (HAL_UART_Transmit(&huart2, data, length, VOFA_UART_TIMEOUT_MS) != HAL_OK)
    ++s_tx_errors[VOFA_PORT_WIFI];
}

uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
  uint8_t data = 0U;

  (void)handle;
  (void)VofaPort_ReadByte(&data);
  return data;
}
