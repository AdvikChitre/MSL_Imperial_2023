// Adapted from code written by Phil Salmony: https://github.com/pms67/HadesFCS/tree/master/Filtering

#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include "Arduino.h"

#define FIRFilterLength 17

struct FIRFilter{
  float buf[FIRFilterLength];
  uint8_t bufIndex;

  float output;
};

void FIRFilterInit(FIRFilter *fir);
float FIRFilterUpdate(FIRFilter *fir, float input);

#endif