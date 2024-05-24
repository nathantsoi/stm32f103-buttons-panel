#ifndef __LOWPASSFILTER_H__
#define __LOWPASSFILTER_H__

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float smooth, beta;
} lp_t;

void lp_filter(lp_t *lp, uint32_t raw);

#ifdef __cplusplus
}
#endif

#endif
