#include <Arduino.h>
#include <stdio.h>

#include "Miscellaneous.h"
#include <stdarg.h>

const char printf_conv_table[] =  "0123456789ABCDEF";
int printf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  int is_spec = 0;
  while (*format != 0) {
    char c = *format;
    if (!is_spec) { // We have no specifier, usual write
      if (c == '%') {
        is_spec = 1;
      } else {
        Serial.print(c);
      }
    } else { // Write with specifier
      char buffer[256];
      unsigned int radix = 10;
      unsigned int number = 0;
      int pos = 0;
      switch (c) {
        case '%':
          Serial.print('%');
          break;
        case 's':
          Serial.print(va_arg(args, char *));
          break;
        case 'g':
          Serial.print(va_arg(args, double));
          break;
        case 'x':
          radix = 16;
        case 'u':
        case 'd':
          number = va_arg(args, unsigned int);
          if ((c == 'd') && ((int)number < 0)) {
            number = -number;
            Serial.print('-');
          }
          do {
            buffer[pos] = printf_conv_table[number % radix];
            number /= radix;
            pos++;
          } while (number > 0);
          for (int i = pos - 1; i >= 0; i--)
            Serial.print(buffer[i]);
          break;
        default:
          break;
      }
      is_spec = 0;
    }
    // Go to next symbol
    format++;
  }
  va_end(args);
  return 0;
}
