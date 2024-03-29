/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    STM32/USARTv1/uart_lld.c
 * @brief   STM32 low level UART driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define USART1_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART1_RX_DMA_STREAM,                     \
                       STM32_USART1_RX_DMA_CHN)

#define USART1_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART1_TX_DMA_STREAM,                     \
                       STM32_USART1_TX_DMA_CHN)

#define USART2_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART2_RX_DMA_STREAM,                     \
                       STM32_USART2_RX_DMA_CHN)

#define USART2_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART2_TX_DMA_STREAM,                     \
                       STM32_USART2_TX_DMA_CHN)

#define USART3_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART3_RX_DMA_STREAM,                     \
                       STM32_USART3_RX_DMA_CHN)

#define USART3_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART3_TX_DMA_STREAM,                     \
                       STM32_USART3_TX_DMA_CHN)

#define USART6_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART6_RX_DMA_STREAM,                     \
                       STM32_USART6_RX_DMA_CHN)

#define USART6_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART6_TX_DMA_STREAM,                     \
                       STM32_USART6_TX_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 UART driver identifier.*/
#if STM32_UART_USE_USART1 || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/** @brief USART2 UART driver identifier.*/
#if STM32_UART_USE_USART2 || defined(__DOXYGEN__)
UARTDriver UARTD2;
#endif

/** @brief USART3 UART driver identifier.*/
#if STM32_UART_USE_USART3 || defined(__DOXYGEN__)
UARTDriver UARTD3;
#endif


/** @brief USART6 UART driver identifier.*/
#if STM32_UART_USE_USART6 || defined(__DOXYGEN__)
UARTDriver UARTD6;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Status bits translation.
 *
 * @param[in] sr        USART SR register value
 *
 * @return  The error flags.
 */
static uartflags_t translate_errors(uint16_t sr) {
  uartflags_t sts = 0;

  if (sr & USART_SR_ORE)
    sts |= UART_OVERRUN_ERROR;
  if (sr & USART_SR_PE)
    sts |= UART_PARITY_ERROR;
  if (sr & USART_SR_FE)
    sts |= UART_FRAMING_ERROR;
  if (sr & USART_SR_NE)
    sts |= UART_NOISE_ERROR;
  if (sr & USART_SR_LBD)
    sts |= UART_BREAK_DETECTED;
  return sts;
}

/**
 * @brief   Puts the receiver in the UART_RX_IDLE state.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void set_rx_idle_loop(UARTDriver *uartp) {
  uint32_t mode;
  
  /* RX DMA channel preparation, if the char callback is defined then the
     TCIE interrupt is enabled too.*/
  if (uartp->config->rxchar_cb == NULL)
    mode = STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CIRC;
  else
    mode = STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CIRC | STM32_DMA_CR_TCIE;
  dmaStreamSetMemory0(uartp->dmarx, &uartp->rxbuf);
  dmaStreamSetTransactionSize(uartp->dmarx, 1);
  dmaStreamSetMode(uartp->dmarx, uartp->dmamode | mode);
  dmaStreamEnable(uartp->dmarx);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_stop(UARTDriver *uartp) {

  /* Stops RX and TX DMA channels.*/
  dmaStreamDisable(uartp->dmarx);
  dmaStreamDisable(uartp->dmatx);
  
  /* Stops USART operations.*/
  uartp->usart->CR1 = 0;
  uartp->usart->CR2 = 0;
  uartp->usart->CR3 = 0;
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_start(UARTDriver *uartp) {
  uint16_t cr1;
  USART_TypeDef *u = uartp->usart;

  /* Defensive programming, starting from a clean state.*/
  usart_stop(uartp);

  /* Baud rate setting.*/
  if (uartp->usart == USART1)
    u->BRR = STM32_PCLK2 / uartp->config->speed;
  else
    u->BRR = STM32_PCLK1 / uartp->config->speed;

  /* Resetting eventual pending status flags.*/
  (void)u->SR;  /* SR reset step 1.*/
  (void)u->DR;  /* SR reset step 2.*/
  u->SR = 0;

  /* Note that some bits are enforced because required for correct driver
     operations.*/
  if (uartp->config->txend2_cb == NULL)
    cr1 = USART_CR1_UE | USART_CR1_PEIE | USART_CR1_TE | USART_CR1_RE;
  else
    cr1 = USART_CR1_UE | USART_CR1_PEIE | USART_CR1_TE | USART_CR1_RE |
          USART_CR1_TCIE;
  u->CR1 = uartp->config->cr1 | cr1;
  u->CR2 = uartp->config->cr2 | USART_CR2_LBDIE;
  u->CR3 = uartp->config->cr3 | USART_CR3_DMAT | USART_CR3_DMAR |
                                USART_CR3_EIE;

  /* Starting the receiver idle loop.*/
  set_rx_idle_loop(uartp);
}

/**
 * @brief   RX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void uart_lld_serve_rx_end_irq(UARTDriver *uartp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_UART_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_UART_DMA_ERROR_HOOK(uartp);
  }
#else
  (void)flags;
#endif

  if (uartp->rxstate == UART_RX_IDLE) {
    /* Receiver in idle state, a callback is generated, if enabled, for each
       received character and then the driver stays in the same state.*/
    if (uartp->config->rxchar_cb != NULL)
      uartp->config->rxchar_cb(uartp, uartp->rxbuf);
  }
  else {
    /* Receiver in active state, a callback is generated, if enabled, after
       a completed transfer.*/
    dmaStreamDisable(uartp->dmarx);
    uartp->rxstate = UART_RX_COMPLETE;
    if (uartp->config->rxend_cb != NULL)
      uartp->config->rxend_cb(uartp);

    /* If the callback didn't explicitly change state then the receiver
       automatically returns to the idle state.*/
    if (uartp->rxstate == UART_RX_COMPLETE) {
      uartp->rxstate = UART_RX_IDLE;
      set_rx_idle_loop(uartp);
    }
  }
}

/**
 * @brief   TX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void uart_lld_serve_tx_end_irq(UARTDriver *uartp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_UART_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_UART_DMA_ERROR_HOOK(uartp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(uartp->dmatx);

  /* A callback is generated, if enabled, after a completed transfer.*/
  uartp->txstate = UART_TX_COMPLETE;
  if (uartp->config->txend1_cb != NULL)
    uartp->config->txend1_cb(uartp);

  /* If the callback didn't explicitly change state then the transmitter
     automatically returns to the idle state.*/
  if (uartp->txstate == UART_TX_COMPLETE)
    uartp->txstate = UART_TX_IDLE;
}

/**
 * @brief   USART common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void serve_usart_irq(UARTDriver *uartp) {
  uint16_t sr;
  USART_TypeDef *u = uartp->usart;
  
  sr = u->SR;   /* SR reset step 1.*/
  (void)u->DR;  /* SR reset step 2.*/
  if (sr & (USART_SR_LBD | USART_SR_ORE | USART_SR_NE |
            USART_SR_FE  | USART_SR_PE)) {
    u->SR = ~USART_SR_LBD;
    if (uartp->config->rxerr_cb != NULL)
      uartp->config->rxerr_cb(uartp, translate_errors(sr));
  }
  if (sr & USART_SR_TC) {
    u->SR = ~USART_SR_TC;

    /* End of transmission, a callback is generated.*/
    if (uartp->config->txend2_cb != NULL)
      uartp->config->txend2_cb(uartp);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_UART_USE_USART1 || defined(__DOXYGEN__)
#if !defined(STM32_USART1_HANDLER)
#error "STM32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_USART1_HANDLER) {

  CH_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD1);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_UART_USE_USART1 */

#if STM32_UART_USE_USART2 || defined(__DOXYGEN__)
#if !defined(STM32_USART2_HANDLER)
#error "STM32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_USART2_HANDLER) {

  CH_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD2);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_UART_USE_USART2 */

#if STM32_UART_USE_USART3 || defined(__DOXYGEN__)
#if !defined(STM32_USART3_HANDLER)
#error "STM32_USART3_HANDLER not defined"
#endif
/**
 * @brief   USART3 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_USART3_HANDLER) {

  CH_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD3);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_UART_USE_USART3 */

#if STM32_UART_USE_USART6 || defined(__DOXYGEN__)
#if !defined(STM32_USART6_HANDLER)
#error "STM32_USART6_HANDLER not defined"
#endif
/**
 * @brief   USART6 IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_USART6_HANDLER) {

  CH_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD6);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_UART_USE_USART6 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if STM32_UART_USE_USART1
  uartObjectInit(&UARTD1);
  UARTD1.usart   = USART1;
  UARTD1.dmarx   = STM32_DMA_STREAM(STM32_UART_USART1_RX_DMA_STREAM);
  UARTD1.dmatx   = STM32_DMA_STREAM(STM32_UART_USART1_TX_DMA_STREAM);
#endif

#if STM32_UART_USE_USART2
  uartObjectInit(&UARTD2);
  UARTD2.usart   = USART2;
  UARTD2.dmarx   = STM32_DMA_STREAM(STM32_UART_USART2_RX_DMA_STREAM);
  UARTD2.dmatx   = STM32_DMA_STREAM(STM32_UART_USART2_TX_DMA_STREAM);
#endif

#if STM32_UART_USE_USART3
  uartObjectInit(&UARTD3);
  UARTD3.usart   = USART3;
  UARTD3.dmarx   = STM32_DMA_STREAM(STM32_UART_USART3_RX_DMA_STREAM);
  UARTD3.dmatx   = STM32_DMA_STREAM(STM32_UART_USART3_TX_DMA_STREAM);
#endif

#if STM32_UART_USE_USART6
  uartObjectInit(&UARTD6);
  UARTD6.usart   = USART6;
  UARTD6.dmarx   = STM32_DMA_STREAM(STM32_UART_USART6_RX_DMA_STREAM);
  UARTD6.dmatx   = STM32_DMA_STREAM(STM32_UART_USART6_TX_DMA_STREAM);
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  uartp->dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;

  if (uartp->state == UART_STOP) {
#if STM32_UART_USE_USART1
    if (&UARTD1 == uartp) {
      bool_t b;
      b = dmaStreamAllocate(uartp->dmarx,
                            STM32_UART_USART1_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_rx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #1", "stream already allocated");
      b = dmaStreamAllocate(uartp->dmatx,
                            STM32_UART_USART1_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_tx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #2", "stream already allocated");
      rccEnableUSART1(FALSE);
      nvicEnableVector(STM32_USART1_NUMBER,
                       CORTEX_PRIORITY_MASK(STM32_UART_USART1_IRQ_PRIORITY));
      uartp->dmamode |= STM32_DMA_CR_CHSEL(USART1_RX_DMA_CHANNEL) |
                        STM32_DMA_CR_PL(STM32_UART_USART1_DMA_PRIORITY);
    }
#endif

#if STM32_UART_USE_USART2
    if (&UARTD2 == uartp) {
      bool_t b;
      b = dmaStreamAllocate(uartp->dmarx,
                            STM32_UART_USART2_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_rx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #3", "stream already allocated");
      b = dmaStreamAllocate(uartp->dmatx,
                            STM32_UART_USART2_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_tx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #4", "stream already allocated");
      rccEnableUSART2(FALSE);
      nvicEnableVector(STM32_USART2_NUMBER,
                       CORTEX_PRIORITY_MASK(STM32_UART_USART2_IRQ_PRIORITY));
      uartp->dmamode |= STM32_DMA_CR_CHSEL(USART2_RX_DMA_CHANNEL) |
                        STM32_DMA_CR_PL(STM32_UART_USART2_DMA_PRIORITY);
    }
#endif

#if STM32_UART_USE_USART3
    if (&UARTD3 == uartp) {
      bool_t b;
      b = dmaStreamAllocate(uartp->dmarx,
                            STM32_UART_USART3_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_rx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #5", "stream already allocated");
      b = dmaStreamAllocate(uartp->dmatx,
                            STM32_UART_USART3_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_tx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #6", "stream already allocated");
      rccEnableUSART3(FALSE);
      nvicEnableVector(STM32_USART3_NUMBER,
                       CORTEX_PRIORITY_MASK(STM32_UART_USART3_IRQ_PRIORITY));
      uartp->dmamode |= STM32_DMA_CR_CHSEL(USART3_RX_DMA_CHANNEL) |
                        STM32_DMA_CR_PL(STM32_UART_USART3_DMA_PRIORITY);
    }
#endif

#if STM32_UART_USE_USART6
    if (&UARTD6 == uartp) {
      bool_t b;
      b = dmaStreamAllocate(uartp->dmarx,
                            STM32_UART_USART6_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_rx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #5", "stream already allocated");
      b = dmaStreamAllocate(uartp->dmatx,
                            STM32_UART_USART6_IRQ_PRIORITY,
                            (stm32_dmaisr_t)uart_lld_serve_tx_end_irq,
                            (void *)uartp);
      chDbgAssert(!b, "uart_lld_start(), #6", "stream already allocated");
      rccEnableUSART6(FALSE);
      nvicEnableVector(STM32_USART6_NUMBER,
                       CORTEX_PRIORITY_MASK(STM32_UART_USART6_IRQ_PRIORITY));
      uartp->dmamode |= STM32_DMA_CR_CHSEL(USART6_RX_DMA_CHANNEL) |
                        STM32_DMA_CR_PL(STM32_UART_USART6_DMA_PRIORITY);
    }
#endif

    /* Static DMA setup, the transfer size depends on the USART settings,
       it is 16 bits if M=1 and PCE=0 else it is 8 bits.*/
    if ((uartp->config->cr1 & (USART_CR1_M | USART_CR1_PCE)) == USART_CR1_M)
      uartp->dmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    dmaStreamSetPeripheral(uartp->dmarx, &uartp->usart->DR);
    dmaStreamSetPeripheral(uartp->dmatx, &uartp->usart->DR);
    uartp->rxbuf = 0;
  }

  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
  usart_start(uartp);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    usart_stop(uartp);
    dmaStreamRelease(uartp->dmarx);
    dmaStreamRelease(uartp->dmatx);

#if STM32_UART_USE_USART1
    if (&UARTD1 == uartp) {
      nvicDisableVector(STM32_USART1_NUMBER);
      rccDisableUSART1(FALSE);
      return;
    }
#endif

#if STM32_UART_USE_USART2
    if (&UARTD2 == uartp) {
      nvicDisableVector(STM32_USART2_NUMBER);
      rccDisableUSART2(FALSE);
      return;
    }
#endif

#if STM32_UART_USE_USART3
    if (&UARTD3 == uartp) {
      nvicDisableVector(STM32_USART3_NUMBER);
      rccDisableUSART3(FALSE);
      return;
    }
#endif

#if STM32_UART_USE_USART6
    if (&UARTD6 == uartp) {
      nvicDisableVector(STM32_USART6_NUMBER);
      rccDisableUSART6(FALSE);
      return;
    }
#endif
  }
}

/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf) {

  /* TX DMA channel preparation and start.*/
  dmaStreamSetMemory0(uartp->dmatx, txbuf);
  dmaStreamSetTransactionSize(uartp->dmatx, n);
  dmaStreamSetMode(uartp->dmatx, uartp->dmamode    | STM32_DMA_CR_DIR_M2P |
                                 STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
  dmaStreamEnable(uartp->dmatx);
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {

  dmaStreamDisable(uartp->dmatx);
  return dmaStreamGetTransactionSize(uartp->dmatx);
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {

  /* Stopping previous activity (idle state).*/
  dmaStreamDisable(uartp->dmarx);

  /* RX DMA channel preparation and start.*/
  dmaStreamSetMemory0(uartp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(uartp->dmarx, n);
  dmaStreamSetMode(uartp->dmarx, uartp->dmamode    | STM32_DMA_CR_DIR_P2M |
                                 STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
  dmaStreamEnable(uartp->dmarx);
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {
  size_t n;

  dmaStreamDisable(uartp->dmarx);
  n = dmaStreamGetTransactionSize(uartp->dmarx);
  set_rx_idle_loop(uartp);
  return n;
}

#endif /* HAL_USE_UART */

/** @} */
