/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#if !defined(alarmevent_H)
#define alarmevent_H
    
#include <device.h>
#include "FreeRTOS.h"
#include "TTerm.h"
    
typedef struct __alarms__ {
uint16_t num;
uint8_t alarm_level;
char* message;
uint32_t timestamp;
uint32_t value;
} ALARMS;


    
BaseType_t ptr_is_in_flash(void* ptr);
void alarm_push_c(uint8_t level, char* message, uint16_t len, int32_t value);
void alarm_push(uint8_t level, const char* message, int32_t value);
uint32_t alarm_get_num();
void alarm_init();
uint32_t alarm_get(uint32_t index, ALARMS * alm);
void alarm_clear();
uint32_t alarm_pop(ALARMS * alm);
uint32_t alarm_free(ALARMS * alm);
uint8_t CMD_alarms(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

#define ALM_PRIO_INFO       0
#define ALM_PRIO_WARN       1
#define ALM_PRIO_ALARM      2
#define ALM_PRIO_CRITICAL   3

#define ALM_NO_VALUE        0x80000000
   
/* ------------------------------------------------------------------------ */
#endif
/* [] END OF FILE */