// Copyright (c) 2019 Sarun Rattanasiri
// under the 3-Clause BSD License
// https://opensource.org/licenses/BSD-3-Clause

#ifndef __DEBOUNCE_H
#define __DEBOUNCE_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifndef BUTTON_DEBOUNCE__CONFIRM
#define BUTTON_DEBOUNCE__CONFIRM 128
#endif

typedef struct _button_debounce__config {
  void (*rose)(uint8_t idx);
  void (*fell)(uint8_t idx);
  uint8_t idx;
} ButtonDebounce_Config;

typedef enum _button_debounce__internal_status {
  _BUTTON_DEBOUNCE__UNKNOWN,
  _BUTTON_DEBOUNCE__UNKNOWN_TO_HIGH,
  _BUTTON_DEBOUNCE__UNKNOWN_TO_LOW,
  _BUTTON_DEBOUNCE__HIGH,
  _BUTTON_DEBOUNCE__LOW,
  _BUTTON_DEBOUNCE__HIGH_TO_LOW,
  _BUTTON_DEBOUNCE__LOW_TO_HIGH,
} _ButtonDebounce_InternalStatus;

typedef struct _button_debounce__state {
  _ButtonDebounce_InternalStatus status;
  uint8_t confirmation_count;
} ButtonDebounce_State;

#define button_debounce__init(state)             \
  do {                                           \
    (state)->status = _BUTTON_DEBOUNCE__UNKNOWN; \
    (state)->confirmation_count = 0;             \
  } while (0)

void button_debounce__sample(ButtonDebounce_Config* config,
                             ButtonDebounce_State* state,
                             uint8_t button_state);

#endif
