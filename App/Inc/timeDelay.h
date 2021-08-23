#ifndef __TIMEDELAY_H
#define __TIMEDELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
void LL_Init10usTick(uint32_t HCLKFrequency);
void timeDelayUs(uint32_t counterUs);
void timeDelayMs(uint32_t counterMs);
uint64_t getCounter(void);

#ifdef __cplusplus
}
#endif

#endif