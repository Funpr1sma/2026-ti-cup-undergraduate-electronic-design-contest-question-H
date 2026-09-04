#ifndef HARDWARE_DEBUG_PRINT_H_
#define HARDWARE_DEBUG_PRINT_H_

#include <stdbool.h>
#include <stdint.h>

void DebugPrint_U32(uint32_t value);
void DebugPrint_I32(int32_t value, bool showPositiveSign);
void DebugPrint_Bool(bool value);
void DebugPrint_Mask8(uint8_t mask);
void DebugPrint_NamedI32(const char *name, int32_t value);

#endif /* HARDWARE_DEBUG_PRINT_H_ */
