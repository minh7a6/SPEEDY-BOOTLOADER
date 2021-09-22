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
#include "stm32f3xx_ll_usart.h"
#include <type_traits>
#include <string>

#define ROMSIZE 0x33400

void SystemClock_Config(void);

extern "C" {
static inline void usartInit(void)
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
    ErrorStatus stat;
    stat = LL_USART_Init(USART2, &USART_InitStruct);
    if(stat != SUCCESS) Error_Handler();
    LL_USART_Enable(USART2);
    __NOP();
}

/**
 * @brief Initialize GPIO
 * 
 */

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
}
const char nodeName[] = "testUavcan";
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


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  const std::optional<board::ArgumentsFromApplication> args =
    kocherga::VolatileStorage<board::ArgumentsFromApplication>(reinterpret_cast<std::uint8_t*>(0x1000'0000)).take();

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  SystemClock_Config();
  LL_Init10usTick(72000000);
  gpioInit();
  // usartInit();
  // Enabling CAN Clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);
  board::ROMDriver rom_backend;
  kocherga::SystemInfo hwInfo;
  uint8_t *uID = (uint8_t*)(UID_BASE);
  uint8_t uIDarr[12] = {0};
  std::memcpy(uIDarr, uID, 12);
  std::copy(std::begin(uIDarr), std::end(uIDarr), std::begin(hwInfo.unique_id));
  hwInfo.node_name = nodeName;
  hwInfo.hardware_version = {01,01};
  kocherga::Bootloader boot(rom_backend, hwInfo, ROMSIZE, args && args->linger);
  board::UAVCANCommunication canDriver;
  std::optional<board::UAVCANCommunication::Bitrate> can_bitrate;
  std::optional<std::uint8_t>                  uavcan_can_version;
  std::optional<kocherga::NodeID>              uavcan_can_node_id;
  if (args)
  {
      if (args->uavcan_can_bitrate_first > 0)
      {
          can_bitrate = board::UAVCANCommunication::Bitrate{args->uavcan_can_bitrate_first, args->uavcan_can_bitrate_second};
      }
      uavcan_can_version = args->uavcan_can_protocol_version;     // Will be ignored if invalid.
      uavcan_can_node_id = args->uavcan_can_node_id;              // Will be ignored if invalid.
  }
  std::string fileName = "Test"; // Testing node Name
  kocherga::can::CANNode canNode(canDriver, hwInfo.unique_id);
  bool res = boot.addNode(&canNode);
  if (args && (args->trigger_node_index < 2) && res) {
    boot.trigger(args->trigger_node_index, 
                  args->file_server_node_id,
                  fileName.length(), 
                  (uint8_t*)fileName.data());
  }


  while(1) {
    const auto uptime = getCounter();
    // LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    if (const auto fin = boot.poll(std::chrono::microseconds(uptime)))
    {
        if (*fin == kocherga::Final::BootApp)
        {
            board::bootApplication();
        }
        if (*fin == kocherga::Final::Restart)
        {
            board::reset();
        }
    }
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
