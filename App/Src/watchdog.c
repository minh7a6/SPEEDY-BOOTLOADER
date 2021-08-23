#include "stm32f3xx.h"


void watchDogInit(uint32_t seconds)
{
    IWDG->KR = 0xCCCCUL; // Enable Watchdog
    IWDG->KR = 0x5555UL; // Enable Reg access
    IWDG->PR = 7;
    IWDG->RLR = (seconds * 32000);
    IWDG->RLR /= (4*(1 << IWDG->PR) *1000);
    IWDG->RLR--;
    while(IWDG->SR != 0);
}

void watchDogRefresh(void)
{
    IWDG->KR = 0xAAAAUL;
}