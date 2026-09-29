#ifndef _TOF_X8_H_
#define _TOF_X8_H_

#include <Arduino.h>

bool TOF_X8_begin();
bool TOF_X8_isInitialized();
bool TOF_X8_readAll(uint16_t *outBuf, size_t outLen);
void TOF_X8_print(const uint16_t *buf);
void TOF_X8_task_callback();

#endif
