#pragma once

#include <stdint.h>

#include "FreeRTOS.h"

namespace System {
class Memory {
 public:
  static void* Malloc(size_t size);
  static void Free(void* block);
};
}  // namespace System
