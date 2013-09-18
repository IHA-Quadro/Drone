#ifndef _AQ_FOURTH_ORDER_FILTER_H_
#define _AQ_FOURTH_ORDER_FILTER_H_

#include "GlobalDefined.h"

struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
};
extern fourthOrderData fourthOrder[4];

float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters);

void setupFourthOrder(void);

#endif