#ifndef BALANCE_SYSTEM_H
#define BALANCE_SYSTEM_H

#include <stdint.h>
#include "balance_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

void balance_system_init(void);
void balance_system_run(Balance_Target_t new_target, uint8_t *is_error, uint8_t *is_first_run);

#ifdef __cplusplus
}
#endif

#endif // BALANCE_SYSTEM_H
