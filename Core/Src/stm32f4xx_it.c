/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines - register-level version
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f4xx_it.h"

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers                     */
/******************************************************************************/
void NMI_Handler(void)
{
  while (1) {}
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/* Không dùng HAL tick nữa. main.c dùng DWT->CYCCNT để lấy thời gian. */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                     */
/******************************************************************************/

/* Không dùng I2C interrupt. LCD I2C đang truyền kiểu blocking bằng thanh ghi. */
void I2C1_EV_IRQHandler(void)
{
  volatile uint32_t tmp;
  tmp = I2C1->SR1;
  tmp = I2C1->SR2;
  (void)tmp;
}

/* Xóa lỗi I2C cơ bản nếu lỡ phát sinh ngắt lỗi. */
void I2C1_ER_IRQHandler(void)
{
  I2C1->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF |
                 I2C_SR1_OVR  | I2C_SR1_PECERR | I2C_SR1_TIMEOUT |
                 I2C_SR1_SMBALERT);
}

/* Không dùng UART interrupt. main.c đọc UART bằng polling trong UART1_PollReceive(). */
void USART1_IRQHandler(void)
{
  if (USART1->SR & USART_SR_RXNE) {
    volatile uint32_t tmp = USART1->DR;
    (void)tmp;
  }
}
