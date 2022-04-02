#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

#define PRINTF_BUF 80 // define the tmp buffer size (change if desired)
void snap_print(const char *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);
  Serial.print(buf);
}