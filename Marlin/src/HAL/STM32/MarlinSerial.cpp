/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"
#include "MarlinSerial.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#if MB_SNAPMAKER
  #include "snapmaker.h"
#endif

#ifndef USART4
  #define USART4 UART4
#endif
#ifndef USART5
  #define USART5 UART5
#endif

#define DECLARE_SERIAL_PORT(ser_num) \
  void _rx_complete_irq_ ## ser_num (serial_t * obj); \
  MSerialT MSerial ## ser_num (true, USART ## ser_num, &_rx_complete_irq_ ## ser_num); \
  void _rx_complete_irq_ ## ser_num (serial_t * obj) { MSerial ## ser_num ._rx_complete_irq(obj); }

#if USING_HW_SERIAL1
  DECLARE_SERIAL_PORT(1)
#endif
#if USING_HW_SERIAL2
  DECLARE_SERIAL_PORT(2)
#endif
#if USING_HW_SERIAL3
  DECLARE_SERIAL_PORT(3)
#endif
#if USING_HW_SERIAL4
  DECLARE_SERIAL_PORT(4)
#endif
#if USING_HW_SERIAL5
  DECLARE_SERIAL_PORT(5)
#endif
#if USING_HW_SERIAL6
  DECLARE_SERIAL_PORT(6)
#endif
#if USING_HW_SERIAL7
  DECLARE_SERIAL_PORT(7)
#endif
#if USING_HW_SERIAL8
  DECLARE_SERIAL_PORT(8)
#endif
#if USING_HW_SERIAL9
  DECLARE_SERIAL_PORT(9)
#endif
#if USING_HW_SERIAL10
  DECLARE_SERIAL_PORT(10)
#endif
#if USING_HW_SERIALLP1
  DECLARE_SERIAL_PORT(LP1)
#endif

#include "MapleFreeRTOS1030.h"

void MarlinSerial::begin(unsigned long baud, uint8_t config) {
  HardwareSerial::begin(baud, config);
  #if MB_SNAPMAKER
    _serial.rx_callback = _rx_callback;
  #else
    // Replace the IRQ callback with the one we have defined
    TERN_(EMERGENCY_PARSER, _serial.rx_callback = _rx_callback);
  #endif
}

// This function is Copyright (c) 2006 Nicholas Zambetti.
void MarlinSerial::_rx_complete_irq(serial_t *obj) {
  // No Parity error, read byte and store it in the buffer if there is room
  unsigned char c;

  if (uart_getc(obj, &c) == 0) {

    uint16_t i;
    if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL)
      i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;
    else
      i = (unsigned int)(obj->rx_head + 1) % sec_rx_size;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != obj->rx_tail) {
      obj->rx_buff[obj->rx_head] = c;
      obj->rx_head = i;
    }

    if (active_ch == MARLIN_SERIAL_CHANNEL_SECOND) {
      return;
    }

    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser.update(static_cast<MSerialT*>(this)->emergency_state, c);
    #endif
  }
}

void MarlinSerial::set_irq_priority(uint32_t prio) {
  /* Must disable interrupt to prevent handle lock contention */
  HAL_NVIC_DisableIRQ(_serial.irq);

  // HAL_UART_Receive_IT(uart_handlers[obj->index], &(obj->recv), 1);

  /* Enable interrupt */
  HAL_NVIC_SetPriority(_serial.irq, prio, UART_IRQ_SUBPRIO);
  HAL_NVIC_EnableIRQ(_serial.irq);
}

int MarlinSerial::_sec_tx_complete_irq(serial_t *obj) {
  // If interrupts are enabled, there must be more data in the output
  // buffer. Send the next byte
  obj->tx_tail = (obj->tx_tail + 1) % SACP_PDU_MAX_SIZE;

  if (obj->tx_head == obj->tx_tail) {
    return -1;
  }

  return 0;
}

// implemented APIs for marlin
int MarlinSerial::peek(void) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    return HardwareSerial::peek();
  }
  else
    return -1;
}

int MarlinSerial::read(void) {
  // original hardware channel
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
      return HardwareSerial::read();
  }
  else
    return -1;
}

void MarlinSerial::flush(void) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    HardwareSerial::flush();
  }
}

size_t MarlinSerial::write(uint8_t c) {
  #if MB_SNAPMAKER
    if (str_buf_index < MARLIN_SINGLE_LOG_BUFFER_SIZE - 1) {
      str_buf[str_buf_index++] = c;
    } else {
      str_buf[str_buf_index] = '\0';
      str_buf_index = 0;
      smprinter.send_log_to_console(str_buf);
      smprinter.send_log_to_host(str_buf);
      return 1;
    }

    if (c == '\n') {
      str_buf[str_buf_index] = '\0';
      str_buf_index = 0;
      smprinter.send_log_to_console(str_buf);
      smprinter.send_log_to_host(str_buf);
    }

    return 1;
  #else
    return HardwareSerial::write(c);
  #endif
}

#if MB_SNAPMAKER
size_t MarlinSerial::write_console(uint8_t c) {
  return HardwareSerial::write(c);
}
#endif

int MarlinSerial::available(void) {
  if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    return HardwareSerial::available();
  }
  else {
    // if current channel is not origianl, return 0
    return 0;
  }
}


// API for second channel
int MarlinSerial::peek_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return -1;

  return HardwareSerial::peek();
}

int MarlinSerial::read_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return -1;

  enableHalfDuplexRx();
  // if the head isn't ahead of the tail, we don't have any characters
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  }
  else {
    unsigned char c = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (uint16_t)(_serial.rx_tail + 1) % sec_rx_size;
    return c;
  }
}

void MarlinSerial::flush_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return;

  HardwareSerial::flush();
}

size_t MarlinSerial::write_sec(uint8_t c) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return 0;


  _written = true;

  uint16_t i = (_serial.tx_head + 1) % sec_tx_size;

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to empty it a bit
  while (i == _serial.tx_tail) {
    // nop, the interrupt handler will free up space for us
  }

  _serial.tx_buff[_serial.tx_head] = c;
  _serial.tx_head = i;

  if (!serial_tx_active(&_serial)) {
    uart_attach_tx_callback(&_serial, _sec_tx_complete_irq);
  }

  return 1;
}

int MarlinSerial::available_sec(void) {
  if (active_ch != MARLIN_SERIAL_CHANNEL_SECOND)
    return 0;

  return ((unsigned int)(sec_rx_size + _serial.rx_head - _serial.rx_tail)) % sec_rx_size;
}


int MarlinSerial::read_multi(uint8_t ch, uint8_t *buffer, uint16_t length) {
  if (ch != active_ch)
    return 0;

  volatile uint16_t size = 0;

  if (ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    size = SERIAL_RX_BUFFER_SIZE;
      if (available() == 0)
        return 0;

      if (available() < length)
        length = available();
  }
  else {
    size = sec_rx_size;

    if (available_sec() == 0)
      return 0;

    if (available_sec() < length)
      length = available_sec();
  }

  for (int i = 0; i < length; i++) {
    buffer[i] = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (uint16_t)(_serial.rx_tail + 1) % size;
  }

  return length;
}


int MarlinSerial::write_multi(uint8_t ch, uint8_t *buffer, uint16_t length) {
  if (ch != active_ch)
    return -1;

  volatile uint16_t buffer_size = 0;
  volatile uint16_t free_size = 0;

  uint16_t head = _serial.tx_head;
  uint16_t tail = _serial.tx_tail;

  if (ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    buffer_size = SERIAL_TX_BUFFER_SIZE;
  }
  else {
    buffer_size = sec_tx_size;
  }

  if (head >= tail) {
    free_size =  buffer_size - 1 - head + tail;
  }
  else {
    free_size =  tail - head - 1;
  }

  if (free_size == 0)
    return free_size;

  if (free_size < length)
    length = free_size;

  for (int i = 0; i < length; i++) {
    head = (_serial.tx_head + 1) % buffer_size;

    while (head == _serial.tx_tail) {
      // nop, the interrupt handler will free up space for us
    }
    _serial.tx_buff[head] = buffer[i];
    _serial.tx_head = head;
  }

  if (!serial_tx_active(&_serial)) {
    if (active_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL)
      uart_attach_tx_callback(&_serial, _tx_complete_irq);
    else
      uart_attach_tx_callback(&_serial, _sec_tx_complete_irq);
  }

  return length;
}


int MarlinSerial::set_active_channel(uint8_t new_ch) {

  if (new_ch == active_ch)
    return 0;

  if (new_ch >= MARLIN_SERIAL_CHANNEL_INVALID)
    return -1;

  // IRQ level of UART is 10, could be disable by below API
  taskENTER_CRITICAL();

  if (new_ch == MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    // save index firstly
    sec_rx_head = _serial.rx_head;
    sec_rx_tail = _serial.rx_tail;
    sec_tx_head = _serial.tx_head;
    sec_tx_tail = _serial.tx_tail;
    // clear signal

    // clear orignal buffer index
    orig_rx_head = 0;
    orig_rx_tail = 0;
    orig_tx_head = 1;
    orig_tx_tail = 0;

    // update current buffer to original
    _serial.rx_buff = _rx_buffer;
    _serial.rx_head = orig_rx_head;
    _serial.rx_tail = orig_rx_tail;
    _serial.tx_buff = _tx_buffer;
    _serial.tx_head = orig_tx_head;
    _serial.tx_tail = orig_tx_tail;
  }
  else {
    // save index firstly
    orig_rx_head = _serial.rx_head;
    orig_rx_tail = _serial.rx_tail;
    orig_tx_head = _serial.tx_head;
    orig_tx_tail = _serial.tx_tail;

    // clear orignal buffer index
    sec_rx_head = 0;
    sec_rx_tail = 0;
    sec_tx_head = 1;
    sec_tx_tail = 0;

    // buffer for second channel
    _serial.rx_buff = sec_rx_buffer;
    _serial.rx_head = sec_rx_head;
    _serial.rx_tail = sec_rx_tail;
    _serial.tx_buff = sec_tx_buffer;
    _serial.tx_head = sec_tx_head;
    _serial.tx_tail = sec_tx_tail;
  }

  active_ch = new_ch;

  taskEXIT_CRITICAL();

  return 0;
}


#define DMA_RX_BUFF_SIZE                1024

static uint8_t *dma_rx_buffer = NULL;
static uint32_t dma_rx_size = 0;
// static DMA_HandleTypeDef dma_handle;
static MarlinSerial *_dma_serial = NULL;
static UART_HandleTypeDef *_dma_serial_handle = NULL;
uint8_t usart_rx_dma_buffer[DMA_RX_BUFF_SIZE];

int MarlinSerial::set_dma_rx_buffer(uint8_t *buffer, uint16_t size) {
  if (dma_rx_buffer || !buffer)
    return -1;

  dma_rx_buffer = buffer;
  dma_rx_size = size;

  return 0;
}

int MarlinSerial::uart2_rx_dma1_init(void) {
  if (_serial.uart != USART2) {
    LOG_E("uart2 rx dma1 init failed, unsupported serial ports\n");
    return -1;
  }

  set_dma_rx_buffer(usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
  _dma_serial = &MSerial2;
  _dma_serial_handle = &MSerial2._serial.handle;

  /* USART2 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t) &(USART2->DR));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)usart_rx_dma_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, sizeof(usart_rx_dma_buffer));

  /* Enable HT & TC interrupts */
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);

  /* DMA interrupt init */
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, HMI_SERIAL_IRQ_PRIORITY, UART_IRQ_SUBPRIO);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  CLEAR_BIT(USART2->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(USART2->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));

  SET_BIT(USART2->CR3, USART_CR3_DMAR);;
  SET_BIT(USART2->CR1, USART_CR1_IDLEIE);

  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

  return 0;
}

void _usart_rx_check(void) {
  static size_t old_pos = 0;
  size_t pos;
  uint16_t index = 0;

  if (!dma_rx_buffer || dma_rx_size <= 0)
    return;

  /* Calculate current position in buffer and check for new data available */
  uint32_t dma_cache_size = LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
  pos = dma_rx_size - dma_cache_size;
  if (pos != old_pos) {                       /* Check change in received data */
    if (pos > old_pos) {                    /* Current position is over previous one */
      // usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
      for (unsigned int i = 0; old_pos + i <  pos; i++) {
        index = (unsigned int)(_dma_serial->_serial.rx_head + 1) % _dma_serial->sec_rx_size;
        if (index != _dma_serial->_serial.rx_tail) {
          _dma_serial->_serial.rx_buff[_dma_serial->_serial.rx_head] = dma_rx_buffer[old_pos + i];
          _dma_serial->_serial.rx_head = index;
        }
      }
    } else {
      // usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
      for (unsigned int i = 0; old_pos + i < dma_rx_size; i++) {
        index = (unsigned int)(_dma_serial->_serial.rx_head + 1) % _dma_serial->sec_rx_size;
        if (index != _dma_serial->_serial.rx_tail) {
          _dma_serial->_serial.rx_buff[_dma_serial->_serial.rx_head] = dma_rx_buffer[old_pos + i];
          _dma_serial->_serial.rx_head = index;
        }
      }

      if (pos > 0) {
        for (unsigned int i = 0; i < pos; i++) {
          index = (unsigned int)(_dma_serial->_serial.rx_head + 1) % _dma_serial->sec_rx_size;
          if (index != _dma_serial->_serial.rx_tail) {
            _dma_serial->_serial.rx_buff[_dma_serial->_serial.rx_head] = dma_rx_buffer[i];
            _dma_serial->_serial.rx_head = index;
          }
        }
      }
    }
    old_pos = pos;                          /* Save current position as old for next transfers */
  }
}

extern "C"
{
  void DMA1_Stream5_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
        _usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear transfer complete flag */
        _usart_rx_check();                       /* Check for data to process */
    }
  }

  void _LL_USART_ClearFlag_IDLE(USART_TypeDef *USARTx)
  {
    __IO uint32_t tmpreg;
    tmpreg = USARTx->SR;
    (void) tmpreg;
    tmpreg = USARTx->DR;
    (void) tmpreg;
  }

  HAL_StatusTypeDef __UART_Transmit_IT(UART_HandleTypeDef *huart)
  {
    uint16_t *tmp;

    /* Check that a Tx process is ongoing */
    if (huart->gState == HAL_UART_STATE_BUSY_TX)
    {
      if (huart->Init.WordLength == UART_WORDLENGTH_9B)
      {
        tmp = (uint16_t *) huart->pTxBuffPtr;
        huart->Instance->DR = (uint16_t)(*tmp & (uint16_t)0x01FF);
        if (huart->Init.Parity == UART_PARITY_NONE)
        {
          huart->pTxBuffPtr += 2U;
        }
        else
        {
          huart->pTxBuffPtr += 1U;
        }
      }
      else
      {
        huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
      }

      if (--huart->TxXferCount == 0U)
      {
        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

        /* Enable the UART Transmit Complete Interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
      }
      return HAL_OK;
    }
    else
    {
      return HAL_BUSY;
    }
  }

  void USART2_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    // if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
    //     LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
    //     usart_rx_check();                       /* Check for data to process */
    // }
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    if ((READ_BIT(USART2->CR1, USART_CR1_IDLEIE) == (USART_CR1_IDLEIE)) && \
        (READ_BIT(USART2->SR, USART_SR_IDLE) == (USART_SR_IDLE))) {
      // __HAL_UART_CLEAR_PEFLAG();
      _LL_USART_ClearFlag_IDLE(USART2);
      _usart_rx_check();
    }

     /* UART in mode Transmitter ------------------------------------------------*/
    if (((USART2->SR & USART_SR_TXE) != RESET) && ((USART2->CR1 & USART_CR1_TXEIE) != RESET))
    {
      __UART_Transmit_IT(_dma_serial_handle);
      return;
    }

    /* UART in mode Transmitter end --------------------------------------------*/
    if (((USART2->SR & USART_SR_TC) != RESET) && ((USART2->CR1 & USART_CR1_TCIE) != RESET))
    {
      // UART_EndTransmit_IT(_dma_serial_handle);

       /* Disable the UART Transmit Complete Interrupt */
      __HAL_UART_DISABLE_IT(_dma_serial_handle, UART_IT_TC);

      /* Tx process is ended, restore huart->gState to Ready */
      _dma_serial_handle->gState = HAL_UART_STATE_READY;
      /*Call legacy weak Tx complete callback*/
      HAL_UART_TxCpltCallback(_dma_serial_handle);
      return;
    }
  }
}

#endif // HAL_STM32
