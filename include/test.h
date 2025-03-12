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

// include some of the globals used (that aren't defined in main.h)
extern volatile uint32_t currentStepSize[2*CHANNELS];
extern uint32_t stepSizes[12];
extern volatile int channelTimes[2*CHANNELS];
extern struct SysState sysState;
extern QueueHandle_t msgOutQ;
extern QueueHandle_t msgInQ;
extern SemaphoreHandle_t CAN_TX_Semaphore;

// functions to benchmark
extern void displayUpdateTask(void * pvParameters);
extern void checkHandshakeTask(void * pvParameters);
extern void receiveCanTask(void * pvParameters);
extern void sendCanTask(void * pvParameters);
extern void scanKeysTask(void * pvParameters);
extern void sampleISR();

// test functions used for profiling
void test_checkHandshakeTask();
void test_displayUpdateTask();
void test_sampleISR();
void test_receiveCanTask();
void test_sendCanTask();
void test_scanKeysTask();

#endif