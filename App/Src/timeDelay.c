#include "stm32f3xx.h"
#include "stm32f3xx_ll_utils.h"
#include "timeDelay.h"
#include "main.h"

volatile uint64_t counter = 0;
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    counter += 10;
}


void LL_Init10usTick(uint32_t HCLKFrequency)
{
    uint32_t prioritygroup = 0x00U;
    if(SysTick_Config(HCLKFrequency / (100000U)) > 0U) Error_Handler();
    prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
}

uint64_t getCounter(void)
{
    return counter;
}

void timeDelayUs(uint32_t counterUs)
{
  uint64_t target = getCounter() + counterUs;
  while(target > getCounter());
}

void timeDelayMs(uint32_t counterMs)
{
  timeDelayUs(counterMs * 1000);
}