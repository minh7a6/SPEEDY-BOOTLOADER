#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void watchDogInit(uint32_t seconds);
void watchDogRefresh(void);

#ifdef __cplusplus
}
#endif

#endif