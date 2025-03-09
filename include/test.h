#ifndef TEST_H
#define TEST_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <knob.h>
#include <drum.h>
#include <cstring>
#include <unordered_set>
#include <ES_CAN.h>
#include "main.h"

// include some of the globals used
extern const int CHANNELS;
extern volatile uint32_t currentStepSize[2*CHANNELS];
extern uint32_t stepSizes[12];
extern volatile int channelTimes[2*CHANNELS];



extern struct SysState sysState;
extern QueueHandle_t msgOutQ;
extern void displayUpdateTask(void * pvParameters);
extern void checkHandshakeTask(void * pvParameters);
extern void sampleISR();

// test functions used for profiling
void test_checkHandshakeTask();
void test_displayUpdateTask();
void test_sampleISR();

#endif