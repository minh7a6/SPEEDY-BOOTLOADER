/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <board.hpp>
#include "timeDelay.h"
#include "watchdog.h"
#include "stm32f3xx_ll_usart.h"


#define ROMSIZE 0x33800
static kocherga_uavcan::HardwareInfo hwInfo;

void SystemClock_Config(void);

static void gpioInit(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    // Enabling GPIOA, B, C Clock
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_InitTypeDef gpioStruct;

  gpioStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
  gpioStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  gpioStruct.Alternate = LL_GPIO_AF_9;
  gpioStruct.Pull = LL_GPIO_PULL_NO;
  gpioStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOA, &gpioStruct);
  
  gpioStruct.Pin = NODE3_Pin|NODE2_Pin|NODE1_Pin;
  gpioStruct.Mode = LL_GPIO_MODE_INPUT;
  gpioStruct.Pull = LL_GPIO_PULL_UP;
  gpioStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOA, &gpioStruct);

  gpioStruct.Pin = NODE4_Pin;
  gpioStruct.Mode = LL_GPIO_MODE_INPUT;
  gpioStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &gpioStruct);

  gpioStruct.Pin = LED_Pin;
  gpioStruct.Mode = LL_GPIO_MODE_OUTPUT;
  gpioStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &gpioStruct);
  
  /*
    * USART2 GPIO Configuration
    *
    * PA2   ------> USART2_TX
    * PA3  ------> USART2_RX
    */
  gpioStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  gpioStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  gpioStruct.Alternate = LL_GPIO_AF_7;
  gpioStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpioStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpioStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &gpioStruct);
}

static inline uint8_t getID(void)
{
  uint32_t res;
  res = LL_GPIO_ReadInputPort(GPIOA);
  uint8_t id = ((res & (NODE4_Pin)) << 3) |
               ((res & (NODE3_Pin)) << 2) |
               ((res & (NODE2_Pin)) << 1) |
               (res & (NODE1_Pin));
  id += 30;
  return res;
}

void usartInit(void)
{
    LL_USART_InitTypeDef USART_InitStruct;

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_Enable(USART2);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  SystemClock_Config();
  LL_Init10usTick(72000000);
  gpioInit();
  // Enabling CAN Clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);

  // set Watchdog 20s
  // watchDogInit(20);
  board::Platform platform;
  board::ROMDriver rom(ROMSIZE);
  hwInfo.major = 12;
  hwInfo.minor = 34;
  uint8_t *uID = (uint8_t*)(UID_BASE);
  uint8_t uIDarr[12];
  std::memcpy(uIDarr, uID, 12);
  std::copy(std::begin(uIDarr), std::end(uIDarr), std::begin(hwInfo.unique_id));
  kocherga::BootloaderController blc(platform, rom, ROMSIZE);
  board::USARTCommunication usart_platform(blc, usartInit, USART2);
  kocherga_ymodem::YModemProtocol protocol(usart_platform);
  kocherga::State state = blc.getState();
  if(state == kocherga::State::ReadyToBoot) {
    board::bootApplication();
  } //Jump to App
  else {
    blc.upgradeApp(protocol);
  }

  while(1) {
    timeDelayMs(1);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  // LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}