#include "system.h"
#include "clock.h"

SystemService system_svc;

uint32_t SystemService::millis(void) {
  return getCurrentMillis();
}
