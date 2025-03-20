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
    uint32_t endTime = micros();
  Serial.println("======== checkHandshakeTask test ========");
  Serial.print("Time: ");
	Serial.print((endTime-startTime)/32);
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
    uint32_t endTime = micros();
  Serial.println("======== displayUpdateTask test ========");
  Serial.print("Time: ");
	Serial.print((endTime-startTime)/32);
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
    uint32_t endTime = micros();
    Serial.println("======== sampleISR test ========");
    Serial.print("Time: ");
    Serial.print((endTime-startTime)/32);
    Serial.print(" us\n");
}

// code to benchmark sampleISR
void test_genBufferTask()
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
      xSemaphoreGive(sampleBufferSemaphore);
      genBufferTask(NULL);
    }

    uint32_t endTime = micros();
    Serial.println("======== genBufferTask test ========");
    Serial.print("Time: ");
    Serial.print((endTime-startTime)/32);
    Serial.print(" us\n");
}

void test_receiveCanTask()
{
  // init worst possible test case here...
  
  // define 32 input messages
  msgInQ = xQueueCreate(32,8);
  uint8_t RX_Message_ISR[8];
  RX_Message_ISR[0] = 'O';
  RX_Message_ISR[1] = 1;
  for(int i = 0; i < 32; i++)
  {
    xQueueSend(msgInQ, RX_Message_ISR, NULL);
  }

  // benchmark
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    receiveCanTask(NULL);
  }
  uint32_t endTime = micros();
  Serial.println("======== receiveCanTask test ========");
  Serial.print("Time: ");
	Serial.print((endTime-startTime)/32);
  Serial.print(" us\n");
}

void test_sendCanTask()
{
  // init worst possible test case here...
  
  // define 32 input messages
  msgOutQ = xQueueCreate(320,8);
  uint8_t TX_Message_ISR[8];
  TX_Message_ISR[0] = 'P';
  TX_Message_ISR[1] = 1;
  TX_Message_ISR[2] = 1;
  for(int i = 0; i < 32; i++)
  {
    xQueueSend(msgOutQ, TX_Message_ISR, NULL);
  }

  // benchmark
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    sendCanTask(NULL);

    // TODO: see if this can be done without including this in the timing
    xSemaphoreGive(CAN_TX_Semaphore);
  }
  uint32_t endTime = micros();
  Serial.println("======== sendCanTask test ========");
  Serial.print("Time: ");
	Serial.print((endTime-startTime)/32);
  Serial.print(" us\n");
}

void test_scanKeysTask()
{
  // init worst possible test case here...
  sysState.stereo = true;
  sysState.BOARD_ID = 0;
  sysState.octaveOverride = false;
  sysState.reverb = true;
  msgOutQ = xQueueCreate(320,8);
  // ensures all key presses are new (results in CAN messages)
  for(int i = 0; i < CHANNELS; i++){ sysState.keyPressed[i] = 0xFFFF;}
  
  // benchmark
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++)
  {
    scanKeysTask(NULL);
  }
  uint32_t endTime = micros();
  Serial.println("======== scanKeysTask test ========");
  Serial.print("Time: ");
	Serial.print((endTime-startTime)/32);
  Serial.print(" us\n");
}