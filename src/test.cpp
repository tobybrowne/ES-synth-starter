#include "test.h"

void test_checkHandshakeTask()
{
  // init worst possible test case here...
  
  // forces a full handshake routine
  sysState.inputs[23] = 1;
  sysState.inputs[27] = 1;

  // increase message buffer (since they won't be being cleared)
  msgOutQ = xQueueCreate(360,8);

  // benchmark
  uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++)
    {
        checkHandshakeTask(NULL);
    }
  Serial.println("======== checkHandshakeTask test ========");
  Serial.print("Time: ");
	Serial.print((micros()-startTime)/32);
  Serial.print(" us\n");
}

void test_displayUpdateTask()
{
  // init worst possible test case here...
  
  // aims to remove any optimisations that occur from 0 values
  for(int i = 0; i < 2*CHANNELS; i++)
  {
    sysState.keyPressed[i] = 0; // every channel is playing a C
  }
  sysState.handshakePending = true;

  // benchmark
  uint32_t startTime = micros();
	for (int iter = 0; iter < 32; iter++)
    {
		displayUpdateTask(NULL);
	}
  Serial.println("======== displayUpdateTask test ========");
  Serial.print("Time: ");
	Serial.print((micros()-startTime)/32);
  Serial.print(" us\n");
}

// code to benchmark sampleISR
void test_sampleISR()
{
  // init worst possible test case here...
  
  // aims to remove any optimisations that occur from 0 values
  for(int i = 0; i < 2*CHANNELS; i++)
  {
    currentStepSize[i] = stepSizes[0]; // every channel is playing a C note
    channelTimes[i] = 10; // midway through it's press
  }

  // benchmark
  uint32_t startTime = micros();
	for (int iter = 0; iter < 32; iter++)
  {
		sampleISR();
	}
  Serial.println("======== sampleISR test ========");
  Serial.print("Time: ");
	Serial.print((micros()-startTime)/32);
  Serial.print(" us\n");
}