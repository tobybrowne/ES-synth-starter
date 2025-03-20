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
#include "test.h"
#include "analog.h"

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc1;

// Interrupt handler for DMA1 channel 3
extern "C" void DMA1_Channel3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_dac1);
}

// extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//   if (htim->Instance == TIM2) {  // Check if it's TIM2
//       Serial.println("TIM2 event triggered!");
//   }
// }

const int SAMPLE_BUFFER_SIZE = 512;
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE/2];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE/2];
volatile bool writeBuffer1 = false;
SemaphoreHandle_t sampleBufferSemaphore; 

#define TESTING

// WE ONLY DO SEMAPHORES AND MUTEXES TO ENSURE NOTHING GETS READ WHILST WE ARE STILL WRITING TO IT
// HENCE WHY AN ATOMIC STORE IS IMPORTANT BUT NOT AN ATOMIC READ
// THE ISR MAY INTERRUPT A PROCESS AT ANY POINTS, ALWAYS REMEMBER

// ISRs INTERRUPTS ALL RTOS TASKS

// currentStepSize can be read by the sampling ISR at any point so it must always be in a valid state

// We use atomic stores, in case the variable being written to is read in an ISR, which may be called mid-way through the writing process.
// An atomic load is used when a variable is being read from which may be being written to at the same time (we don't have this case in our code)

// We use VOLATILE for variables which are modified in an ISR and read in an RTOS task

  // only written to in handshake check (done atomically)
  // read from in scanKeysTask
  // last two are west and east (they've both been turned on)
  int outBits[7] = {0, 0, 0, 1, 1, 1, 1}; // thread-safe
  
//Constants
  const uint32_t interval = 100; //Display update interval

  // CAN message buffers (these are thread safe!)
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;

  // controls acccess to the CAN mailboxes (hardware has 3)
  SemaphoreHandle_t CAN_TX_Semaphore;

  // store device state  
  SysState sysState;

  // knob inits
  Knob knob4 = Knob(0, 12, 5); // volume
  Knob octaveKnob = Knob(0, 8, 3);
  Knob knob2 = Knob(0, 2, 0);
  Knob InstrumentKnob = Knob(0, 1, 1);

  uint8_t RX_Message_Temp[8] = {0};

  // TODO: what variables should be volatile?
  // stores the current step sizes that should be played
  // first half stores internal key presses, second half stores external key presses
  // SHOULD JUST BE USED TO TRANSFER FREQS BETWEEN SCANKEYS AND SAMPLEISR
  volatile uint32_t currentStepSize[2*CHANNELS];

  // how long since the key was pressed
  // stores an int from 0 -> DAMPER_RESOLUTION
  float DAMPER_RESOLUTION = 30;
  volatile int channelTimes[2*CHANNELS] = {0};
  volatile int releaseTimes[2*CHANNELS] = {0};

  // TODO: make these memory safe by moving into sysState (described in Lab2)
  // stores the last sent/received CAN message
  // only accessed in CAN RX task (mem safe)

  // frequencies for each key at octave 0
  const int keyFreqs[12] = {65, 69, 73, 78, 82, 87, 93, 98, 104, 110, 116, 123};
  
  // stores the step sizes corresponding with each key at octave 0
  uint32_t stepSizes[12];

  // stores sine wave with SINE_RESOLUTION x-values with amplitude -2048 to 2048
  const int SINE_RESOLUTION_BITS = 10;
  const int SINE_RESOLUTION = 1 << SINE_RESOLUTION_BITS;  // 2^SINE_RESOLUTION_BITS

  int sineLookup[SINE_RESOLUTION]; // populated in setup, only read from SampleISR

//Display driver objects
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// define timer object
HardwareTimer sampleTimer(TIM1);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN,value);
  digitalWrite(REN_PIN,HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN,LOW);
}

void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);

  bool bit0 = rowIdx & 0b001;
  bool bit1 = rowIdx & 0b010;
  bool bit2 = rowIdx & 0b100;

  digitalWrite(RA0_PIN, bit0);
  digitalWrite(RA1_PIN, bit1);
  digitalWrite(RA2_PIN, bit2);

  digitalWrite(REN_PIN, HIGH);
}

std::bitset<4> readCols()
{
  std::bitset<4> return_set;
  return_set[0] = digitalRead(C0_PIN);
  return_set[1] = digitalRead(C1_PIN);
  return_set[2] = digitalRead(C2_PIN);
  return_set[3] = digitalRead(C3_PIN);

  return return_set;
}

// task to decode CAN messages in the RX buffer and play corresponding notes
// READS: RX_Message, BOARD_ID, rightBoard, handshakePending
// WRITES: octaveOverride, RX_Message, outBits, sysState (NO SEMAPHORE :/), ID_RECV
void receiveCanTask(void * pvParameters)
{
  uint8_t RX_Message[8] = {0};
  int octaveCurrent;

  while(1)
  {
    // blocks until data is available
    // yields the CPU to other tasks in the meantime
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    uint8_t action = RX_Message[0];

    // TODO: remove after debugging
    RX_Message_Temp[0] = RX_Message[0];
    RX_Message_Temp[1] = RX_Message[1];
    RX_Message_Temp[2] = RX_Message[2];
    
    // octave change
    if(RX_Message[0] == 'O')
    {
      sysState.octaveOverride = true; // prevents these changes also being broadcasted on CAN
      int8_t octave = (int8_t)RX_Message[1]; // needs to be signed
      octaveCurrent = octaveKnob.getValue();
      octaveKnob.setValue(octaveCurrent + (int)octave);
    }

    // set stereo balance
    if(RX_Message[0] == 'B')
    {
      int stereoBalance = RX_Message[1];
      int leftVolume = 8 - (stereoBalance / 2);
      int rightVolume = stereoBalance / 2;

      // only the right board will receive this but we provide for completeness...
      if(sysState.BOARD_ID == 0) 
      {
        sysState.volume = leftVolume;
      }
      else if(sysState.rightBoard)
      {
        sysState.volume = rightVolume;
      }
    }

    // receive ID during handshake
    if(RX_Message[0] == 'I')
    {
      if(sysState.handshakePending)
      {
        sysState.ID_RECV = RX_Message[1];
      }
    }
    else if(RX_Message[0] == 'E')
    {
      // turn east back on
      __atomic_store_n(&outBits[6], 1, __ATOMIC_RELAXED);
    }

    if(!sysState.sender)
    {
      uint8_t octave = RX_Message[1];
      uint8_t key = RX_Message[2];
      uint32_t stepSize = stepSizes[key] << octave;

      // key press
      if(RX_Message[0] == 'P')
      {
        for(int i = CHANNELS; i < 2*CHANNELS; i++)
        {
          // can only play if we have available channels
          if(sysState.keyPressed[i] == 0xFFFF)
          {
            __atomic_store_n(&channelTimes[CHANNELS + i], 0, __ATOMIC_RELAXED);
            sysState.keyPressed[i] = (octave << 8) | key;
            break;
          }
        } 
      }

      // key release
      if(RX_Message[0] == 'R')
      {
        for(int i = CHANNELS; i < 2*CHANNELS; i++)
        {
          if(sysState.keyPressed[i] == (octave << 8) | key)
          {
            sysState.keyPressed[i] = 0xFFFF;
            break;
          }
        } 
      }
    }

    // release systate mutex
    xSemaphoreGive(sysState.mutex);

    #ifdef TESTING
    break;
    #endif
  }
}

// runs whenever a CAN message is received
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// called to send an ID to another board for handshaking
void sendID(int boardID)
{
  uint8_t TX_Message[8] = {0};
  // send a "release" CAN message
  TX_Message[0] = 'I';
  TX_Message[1] = boardID;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void sendKeyPress(char action, int octave, int keyPressed)
{
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = action;
  TX_Message[1] = octave;
  TX_Message[2] = keyPressed;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void sendEndHandshake()
{
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = 'E';
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

// send a +N octave message
void sendChangeOctave(int octaveChange)
{
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = 'O';
  TX_Message[1] = octaveChange;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

// takes in value from 0 (left) to 16 (right)
void sendStereoBalance(int stereoBalance)
{
  uint8_t TX_Message[8] = {0};
  // send CAN message
  TX_Message[0] = 'B';
  TX_Message[1] = stereoBalance;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

// sets speaker voltage 22,000 times per sec
// TODO: read systate and currentStepSize atomically
// void sampleISR()
// {
//     static uint32_t phaseAcc[CHANNELS * 2] = {0};
//     int16_t Vout = 0;
//     int waveType = waveKnob.getValue();

//     // play drum overlay
//     if (InstrumentKnob.getValue() == 0)
//     {
//       Vout = drum();
//     }
 
//     for (int i = 0; i < CHANNELS * 2; i++)
//     {
//       if (currentStepSize[i] == 0) { continue; };

//       phaseAcc[i] += currentStepSize[i];
//       int16_t v_delta = 0;

//       // Generate waveform based on waveType
//       if (waveType == 0) // Sawtooth Wave
//       {
//           v_delta = (phaseAcc[i] >> 20) - 2048;
//       }
//       else if (waveType == 1) // Sine Wave
//       {
//           int angle = (phaseAcc[i] >> (32 - SINE_RESOLUTION_BITS)) & 0xFF; 
//           v_delta = sineLookup[angle];
//       }
//       else if (waveType == 2) // Square Wave
//       {
//           v_delta = (phaseAcc[i] > (1 << 31)) ? 2047 : -2048;
//       }

//       // Apply damper effect
//       float newValue = 1.0f - (channelTimes[i] * (1.0f / DAMPER_RESOLUTION));
//       v_delta *= newValue;

//       Vout += v_delta;
//     }
    
//     // Apply volume control using logarithmic tapering
//     Vout = Vout >> (8 - volumeKnob.getValue());

//     // Ensure Vout stays within 0-4095 range for 12-bit DAC
//     Vout = constrain(Vout + 2048, 0, 4095);
    
//     // write DAC
//     HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Vout);
// }

void sampleISR()
{
  static uint32_t readCtr = 0;

  if (readCtr == SAMPLE_BUFFER_SIZE/2)
  {
    readCtr = 0;
    writeBuffer1 = !writeBuffer1;
    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
  }
    
  if (writeBuffer1)
  {
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sampleBuffer0[readCtr++]);
  }
  else
  {
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sampleBuffer1[readCtr++]);
  }
}

// attach/detach sampling ISR
void toggleSampleISR(bool on)
{
  // if(on)
  // { 
  //   // initialise sampling interrupt (22,000 times a sec)
  //   sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  //   sampleTimer.attachInterrupt(sampleISR);
  //   sampleTimer.resume();
  // }
  // else
  // {
  //   sampleTimer.detachInterrupt();
  // }
}

// task to check handshake inputs every 20ms
// TODO: do we need to obtain mutex if we just read?
void checkHandshakeTask(void * pvParameters)
{
  // stores last state of east/west
  // forces a check on startup
  int lastWest = -1;
  int lastEast = -1;
  int westDetect;
  int eastDetect;
  int lastSender = -1;

  #ifndef TESTING
  const TickType_t xFrequency = 150/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #endif

  while(1)
  {
    #ifndef TESTING
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
    #endif

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    // check CAN inputs for keyboards on left and right
    westDetect = !sysState.inputs[23];
    eastDetect = !sysState.inputs[27];

    // // east LOW to HIGH
    // if(lastEast == 0 && eastDetect == 1)
    // {
    //   // // keyboard plugged into right
    //   // if(sysState.BOARD_ID != -1)
    //   // {
    //   //   // turn east off
    //   //   __atomic_store_n(&outBits[6], 0, __ATOMIC_RELAXED);
    //   //   setOutMuxBit(HKOE_BIT, LOW);

    //   //   // break for 1.5 secs to allow east signal to propagate
    //   //   sysState.handshakePending = true;
    //   //   sysState.ID_RECV = -1;
    //   //   xSemaphoreGive(sysState.mutex);

    //   //   #ifndef TESTING
    //   //   vTaskDelay(pdMS_TO_TICKS(000));
    //   //   #endif

    //   //   xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    //   //   sysState.handshakePending = false;

    //   //   // send the board it's id
    //   //   sendID(sysState.BOARD_ID+1);
    //   // }
    // }
    // // east HIGH to LOW
    // else if(lastEast == 1 && eastDetect == 0)
    // { 
    //   // keyboard on right removed
    //   // DO NOTHING
    // }
    // // west LOW to HIGH
    // if(lastWest == 0 && westDetect == 1)
    // {
    //   // keyboard is plugged into left
    //   // DO NOTHING HANDSHAKE HASNT STARTED
    //   // OR
    //   // signals being reset after after handshake
    //   // DO NOTHING
    // }

    // west HIGH to LOW
    // keyboard on left unplugged
    // OR keyboard on left finished handshake
    // OR startup of the leftmost board
    // this assumes we dont plug a keyboard in when the rest are in handshake mode (cos west would then be low)
    if((lastWest == 1 && westDetect == 0) || (lastWest == -1 && westDetect == 0))
    {
      // keyboard on left unplugged
      // START HANDSHAKE
      // OR
      // keyboard on left finished handshake
      // START HANDSHAKE

      // turn east off
      __atomic_store_n(&outBits[6], 0, __ATOMIC_RELAXED);
      setOutMuxBit(HKOE_BIT, LOW);

      // wait for CAN message to be received
      // we yield CPU access so other tasks can run in this time
      sysState.handshakePending = true;
      sysState.ID_RECV = -1;
      xSemaphoreGive(sysState.mutex);

      //  TODO: how do we characterise this in a test?
      // TODO: could probably remove this (or shorten it) if CAN_RX was a higher priority
      #ifndef TESTING
      vTaskDelay(pdMS_TO_TICKS(150));
      #endif

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      sysState.handshakePending = false;
      
      sysState.BOARD_ID = sysState.ID_RECV;
      sysState.sender = true;
      if(sysState.ID_RECV == -1) // this is the left-most board
      {
        sysState.BOARD_ID = 0;
        sysState.sender = false;
        sysState.rightBoard = false;
      }
      
      octaveKnob.setValue(sysState.BOARD_ID);
      sysState.octaveOverride = true;
  
      // send board it's new ID
      // TODO: need to get rid of this
      // delay(500);
      sendID(sysState.BOARD_ID+1);

      // if you are the rightmost board then finish the handshaking sequence
      if(!eastDetect)
      {
        if(sysState.BOARD_ID != 0)
        {
          sysState.rightBoard = true;
        }
        
        // send a "end handshake" CAN message
        // sendEndHandshake();

        // in stereo mode both left and right boards receive
        if(sysState.stereo)
        {
          sysState.sender = false;
        }
      }
    }

    // if a change of board role...
    if(lastSender != sysState.sender)
    {
      #ifndef TESTING
      toggleSampleISR(!sysState.sender); // senders don't need this ISR
      #endif
      lastSender = sysState.sender;
    }
    lastWest = westDetect;
    lastEast = eastDetect;

    // release systate mutex
    xSemaphoreGive(sysState.mutex);

    #ifdef TESTING
    break;
    #endif
  }
}

// -100 to 100 for each
void readJoystick(int* horiz, int* vert)
{  
  // taskENTER_CRITICAL();

  // Start ADC conversion for horizontal (X) axis
  HAL_ADC_Start(&hadc1);
  int x_axis = HAL_ADC_GetValue(&hadc1); // right 500 left 3650
  int y_axis = HAL_ADC_GetValue(&hadc1); // 510 top 3510 bottom
  HAL_ADC_Stop(&hadc1);

  y_axis = constrain(y_axis, 410, 3510);
  *vert = map(y_axis, 410, 3510, 100, -100);
  if(*vert < 10 && *vert > -10){ *vert = 0; }

  // TODO: add deadzone to x axis
  x_axis = constrain(x_axis, 50, 3650);
  *horiz = map(x_axis, 50, 3650, 100, -100);

  // taskEXIT_CRITICAL();
}

// task to check keys pressed runs every 20ms
void scanKeysTask(void * pvParameters)
{
  #ifndef TESTING
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #endif

  // stores latest key presses before being stored globally
  uint16_t keyPressedLocal[CHANNELS*2];

  int octave = 0;
  int lastOctave = -1;

  int joystick_vert;
  int joystick_horiz;
  int last_joystick_horiz = -1000; // forces an update on startup

  while(1)
  {
    #ifndef TESTING
    // ensures we sample keyboard only every 50ms
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif
    
    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    // ensures we only access the actual variable once per loop
    uint32_t currentStepSize_Local[2*CHANNELS] = {0};

    readJoystick(&joystick_horiz, &joystick_vert);
    // TODO: needs bigger deadzone
    joystick_horiz = 0;
    joystick_vert = 0;

    if(joystick_horiz != last_joystick_horiz)
    {
      // sends message to right board 
      if(sysState.BOARD_ID == 0 && sysState.stereo)
      {
        // Scale from [-100, 100] to [0, 16]
        int stereoBalance = constrain(joystick_horiz, -100, 100);
        
        stereoBalance = map(stereoBalance, -100, 100, 0, 16);

        // send message to right board
        sendStereoBalance(stereoBalance);

        int leftVolume = 8 - (stereoBalance / 2);

        sysState.volume = leftVolume;
      }
      last_joystick_horiz = joystick_horiz;
    }

    // increment all times in channelTimes
    int newValue;
    for(int i = 0; i < CHANNELS*2; i++)
    {
      newValue = channelTimes[i] + 1;
      // if(newValue > DAMPER_RESOLUTION){ newValue = DAMPER_RESOLUTION; }
      __atomic_store_n(&channelTimes[i], newValue, __ATOMIC_RELAXED);
    }

    // init all internal channels to 0xFFFF
    for(int i = 0; i < CHANNELS; i++){ keyPressedLocal[i] = 0xFFFF;}
  
    // read all 32 inputs into sysState.inputs and keyPressedLocal
    int ch = 0;
    for(int i = 0; i < 8; i++)
    {
      setRow(i);
      if(i < 7) { digitalWrite(OUT_PIN, outBits[i]); } // reset MUX bits
      delayMicroseconds(3);
      std::bitset<4> columns = readCols();
      int start_id = i * 4;
      int index;
      for(int j = 0; j < 4; j++)
      {
        index = start_id + j;
        sysState.inputs[index] = columns[j];

        // move key data into keyPressedLocal
        if(index < 12)
        {
          if(sysState.inputs[index] == 0) // if key pressed...
          {
            // only finite key presses at a time
            if(ch < CHANNELS)
            {
              keyPressedLocal[ch] = (octave << 8) | index;
              ch++;
            }
          }  
        }
      }  
    }

    // override actual key presses - simulate all keys being pressed
    #ifdef TESTING
    for(int i = 0; i < CHANNELS; i++){ sysState.keyPressed[i] = 0x0000;}
    #endif

    // manage sending octave change messages
    octave = octaveKnob.getValue();
    if((octave != lastOctave) && lastOctave != -1)
    {
      if(sysState.octaveOverride)
      {
        sysState.octaveOverride = false; // reset flag
        lastOctave = octave;
      }
      else
      {
        // send a +N octave message
        sendChangeOctave(octave - lastOctave);
        lastOctave = octave;
      }
    }

    // check for key releases
    std::unordered_set<uint8_t> localKeys; // cache key indexes for quick lookup
    for (int j = 0; j < CHANNELS; j++)
    {
      localKeys.insert(keyPressedLocal[j] & 0xFF);
    }
    for (int i = 0; i < CHANNELS; i++)
    {
      uint8_t keyIndex = sysState.keyPressed[i] & 0xFF;
      uint8_t octave = (sysState.keyPressed[i] >> 8) & 0xFF;

      // if keyIndex is NOT in localKeys itss released
      if (localKeys.find(keyIndex) == localKeys.end())
      {
        if(sysState.sender)
        {
            sendKeyPress('R', octave, keyIndex);
        }
      }
    }

    // check for new presses
    std::unordered_set<uint8_t> activeKeys; // cache key indexes for quick lookup
    for (int j = 0; j < CHANNELS; j++)
    {
      activeKeys.insert(sysState.keyPressed[j] & 0xFF);
    }
    for (int i = 0; i < CHANNELS; i++)
    {
      if (keyPressedLocal[i] == 0xFFFF) continue;  // skip empty keys

      uint8_t keyIndex = keyPressedLocal[i] & 0xFF;

      // if keyIndex is NOT found in activeKeys its a new press
      if (activeKeys.find(keyIndex) == activeKeys.end())
      {
        if (sysState.sender)
        {
          sendKeyPress('P', (keyPressedLocal[i] >> 8) & 0xFF, keyIndex);
        }

        __atomic_store_n(&channelTimes[i], 0, __ATOMIC_RELAXED);
      }
    }

    // check knob rotation
    octaveKnob.updateQuadInputs(sysState.inputs[14], sysState.inputs[15]);
    knob2.updateQuadInputs(sysState.inputs[16], sysState.inputs[17]);
    InstrumentKnob.updateQuadInputs(sysState.inputs[18], sysState.inputs[19]); 
    knob4.updateQuadInputs(sysState.inputs[12], sysState.inputs[13]);

    sysState.volume = knob4.getValue();
    __atomic_store_n(&sysState.waveType, knob2.getValue(), __ATOMIC_RELAXED);

  
    // if(!sysState.stereo) // no knob volume control in stereo mode
    // {
      
    // }

    // store key presses in sysState and manage reverb
    for(int i = 0; i < 2*CHANNELS; i++)
    {
      // key is off
      // KEY IS TURNED OFF IN ADSR FUNCTION!
      if(keyPressedLocal[i] == 0xFFFF)
      {
        if(releaseTimes[i] == 0)
        {
          releaseTimes[i] = channelTimes[i];
        }
      }

      // key is on
      else
      {
        sysState.keyPressed[i] = keyPressedLocal[i];
      }

      // if(keyPressedLocal[i] != 0xFFFF)
      // {
      //   sysState.keyPressed[i] = keyPressedLocal[i];
      // }
      // THIS NEVER TURNS KEYS OFF!!!



      // sysState.keyPressed[i] = keyPressedLocal[i];

      // if(sysState.reverb)
      // {
      //   if(keyPressedLocal[i] != 0xFFFF)
      //   {
      //     sysState.keyPressed[i] = keyPressedLocal[i];
      //   }
      //   else if(channelTimes[i] > DAMPER_RESOLUTION - 1)
      //   {
      //     sysState.keyPressed[i] = 0xFFFF;
      //   }
      // }
      // else
      // {
      //   sysState.keyPressed[i] = keyPressedLocal[i];
      // }
    }

    // convert key presses to step sizes (both internal and external)
    for(int i = 0; i < 2*CHANNELS; i++)
    {
      if(sysState.keyPressed[i] != 0xFFFF)
      {
        uint8_t keyIndex = sysState.keyPressed[i] & 0xFF;
        uint8_t octave = (sysState.keyPressed[i] >> 8) & 0xFF;
        currentStepSize_Local[i] = (stepSizes[keyIndex] << octave) * (1 + static_cast<float>(joystick_vert)/150);
      }
    }

    // release systate mutex
    xSemaphoreGive(sysState.mutex);
  
    // atomic write to currentStepSize global
    // if the ISR only receives half of the updated values thats fine, but we can't have half-written elements
    for(int i = 0; i < 2*CHANNELS; i++)
    {
      __atomic_store_n(&currentStepSize[i], currentStepSize_Local[i], __ATOMIC_RELAXED);
    }

    #ifdef TESTING
    break;
    #endif
  } 
}

// task to update OLED display every 100ms
void displayUpdateTask(void * pvParameters)
{
  #ifndef TESTING
  const TickType_t xFrequency = 200/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #endif

  // strings used for display
  const char* keyNames [12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  const char* waveTypes[3] = {"Saw", "Sine", "Square"};
  const char* instrumentTypes[2] = {"Drums", "Piano"};

  while(1)
  {
    #ifndef TESTING
    // ensures we update screen every 100ms
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    
    // display notes pressed on current keyboard
    u8g2.setCursor(2,10);
    u8g2.print("Keys: ");

    // Print the keys that are pressed
    for (int i = 0; i < CHANNELS; i++) {
      if (sysState.keyPressed[i] != 0xFFFF) {
        
        u8g2.print(keyNames[sysState.keyPressed[i] & 0xFF]);
        u8g2.print(" ");
      }
    }

    // Determine the width of the display
    int displayWidth = u8g2.getDisplayWidth();
    
    int textWidth = u8g2.getStrWidth("00"); // RECV and SEND have same width
    u8g2.setCursor(displayWidth - textWidth, 10);
    u8g2.print(sysState.BOARD_ID);

    // display octave of current keyboard
    u8g2.setCursor(2,20);
    u8g2.print("Oct: ");
    u8g2.print(octaveKnob.getValue());

    // display wave type of current keyboard
    textWidth = u8g2.getStrWidth("Wave: ") + u8g2.getStrWidth(waveTypes[sysState.waveType]);
    u8g2.setCursor(displayWidth - textWidth, 20);
    u8g2.print("Wave: ");
    u8g2.print(waveTypes[sysState.waveType]);

    // temporarily removed
    // textWidth = u8g2.getStrWidth("Instr: ") + u8g2.getStrWidth(instrumentTypes[InstrumentKnob.getValue()]);
    // u8g2.setCursor(displayWidth - textWidth, 30);
    // u8g2.print("Instr: ");
    // u8g2.print(instrumentTypes[InstrumentKnob.getValue()]);

    // display volume of current keyboard
    u8g2.setCursor(2,30);
    u8g2.print("Vol: ");
    u8g2.print(sysState.volume);

    // display last received CAN message
    u8g2.setCursor(63,30);
    u8g2.print((char) RX_Message_Temp[0]);
    u8g2.print(RX_Message_Temp[1]);
    u8g2.print(RX_Message_Temp[2]);

    if(sysState.handshakePending)
    {
      textWidth = u8g2.getStrWidth("HAND");
      u8g2.setCursor(displayWidth - textWidth, 10);
      u8g2.print("HAND");
    }

    // push screen buffer to display
    u8g2.sendBuffer();

    // toggle lED
    digitalToggle(LED_BUILTIN);

    // release systate mutex
    xSemaphoreGive(sysState.mutex);

    #ifdef TESTING
    break; // only run 1 iter for testing
    #endif
  } 
}

// scan TX mailbox and wait until a mailbox is available
void sendCanTask (void * pvParameters) {
	uint8_t msgOut[8];
	while (1)
  {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);

		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);

		CAN_TX(0x123, msgOut);

    #ifdef TESTING
    break;
    #endif
	}
}

// frees CAN semaphore when mailbox is available
void CAN_TX_ISR (void)
{
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

// outputs timing data every 5 seconds
void TaskMonitor(void *pvParameters) {
  char statsBuffer[512];

  while (1) {
      vTaskGetRunTimeStats(statsBuffer);
      Serial.println("Task Execution Time Stats:");
      Serial.println(statsBuffer);

      vTaskDelay(pdMS_TO_TICKS(5000));  // Print stats every 5 seconds
  }
}

// need to test different parameters for this
// piano? / strings / guitar / drums
int attackTime = 20;    
int decayTime = 20;     
float sustainLevel = 1; // number from 0 to 1
int releaseTime = 200;  

// TODO: currently take 2ms!!! // make faster
float applyADSR(int i)
{
  unsigned long currentTime = channelTimes[i];  
  int envelope = 0;
  float float_envelope = 0;

  // key is in release stage
  if(releaseTimes[i] > 0 )
  {
    float_envelope = (sustainLevel) - (sustainLevel * ((currentTime - releaseTimes[i]))/(releaseTime));
    // float_envelope = (float)envelope / 100;
    
    if(float_envelope < 0.01)
    {
      // turn key off
      // TODO: does keyPressed and releaseTimes need atomic access?
      __atomic_store_n(&sysState.keyPressed[i], 0xFFFF, __ATOMIC_RELAXED);
      releaseTimes[i] = 0;
    }
  }
  else
  {
    // attack: increase amp gradually 
    if (currentTime < attackTime)
    {
      float_envelope = (float)currentTime / attackTime;  
    }
    // decay: decrease to sustain amp
    else if (currentTime < attackTime + decayTime)
    {
      int decayProgress = currentTime - attackTime;
      return 1.0f - (1.0f - sustainLevel) * ((float)decayProgress / decayTime);
    }
    // sustain 
    else
    {
      float_envelope = sustainLevel;
    }
  }

  // Ensure envelope stays within bounds (0 to 1)
  if(float_envelope > 1) float_envelope = 1;
  if(float_envelope < 0) float_envelope = 0;
  return float_envelope;
}


// auto writes when buffer is available
void genBufferTask(void *pvParameters)
{
  // only 1 channel
  uint32_t phaseAcc[CHANNELS * 2] = {0};
  uint16_t Vout = 0; //Calculate one sample
  uint16_t v_delta = 0;

  while(1)
  {
    xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
    int waveType = sysState.waveType;
    for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE/2; writeCtr++)
    {
      v_delta = 0;
      Vout = 0;
  
      // loop through internal channels
      for (int i = 0; i < 2*CHANNELS; i++)
      {
        if (currentStepSize[i] == 0) { continue; };

        phaseAcc[i] += currentStepSize[i];

        // Generate waveform based on waveType
        if (waveType == 0) // Sawtooth Wave
        {
          v_delta = (phaseAcc[i] >> 20) - 2048;
        }
        else if (waveType == 1) // Sine Wave
        {
          // int angle = (phaseAcc[i] >> (32 - SINE_RESOLUTION_BITS)) & 0xFF; 
          int angle = (phaseAcc[i] >> (32 - SINE_RESOLUTION_BITS)) & ((1 << SINE_RESOLUTION_BITS) - 1);
          v_delta = sineLookup[angle];
        }
        else if (waveType == 2) // Square Wave
        {
          v_delta = (phaseAcc[i] > (1 << 31)) ? 2047 : -2048;
        }

        // // Apply damper effect
        // float newValue = 1.0f - (channelTimes[i] * (1.0f / DAMPER_RESOLUTION));
        // v_delta *= newValue;

        Vout += v_delta * applyADSR(i);
        //Vout += v_delta;
      }

      // Apply volume control using logarithmic tapering
      Vout = Vout >> (12 - sysState.volume);
      // Vout = Vout;

      // Normalize the volume when multiple keys are pressed
      // if (activeKeys > 0) {
      //   Vout /= activeKeys;
      // }

      // Vout = constrain(Vout + 2048, 0, 4095);
      // Vout += 2048;
      // if (Vout > 4095) Vout = 4095;
      // if (Vout < 0) Vout = 0;

      if (writeBuffer1)
      {
        sampleBuffer1[writeCtr] = Vout;
      }
      else
      {
        sampleBuffer0[writeCtr] = Vout;
      } 
    }

    #ifdef TESTING
    break;
    #endif
  }
}

void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure PB7 as Output */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  // Configure DMA for DAC channel 1.
  hdma_dac1.Instance = DMA1_Channel3;
  // hdma_dac1.Init.Request = DMA_REQUEST_1; // This now resolves to DMA_REQUEST_1 if not defined
  hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_dac1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_dac1);

  // Link the DMA handle to the DAC handle.
  __HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac1);
}

void MX_TIM2_Init(void)
{
    /* Enable TIM2 Clock */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Configure TIM2 for DAC trigger */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;  // 1 MHz tick
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 100;  // Adjust for waveform frequency
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure TIM2 to trigger DAC */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_DAC_Init(void)
{
    /* Enable DAC Clock */
    __HAL_RCC_DAC1_CLK_ENABLE();

    /* Configure DAC */
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure DAC Channel 1 */
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

// extern "C" void TIM2_IRQHandler(void) {
//   if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
//       __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
//       Serial.println("TIM2 event triggered!\n");
//   }
// }

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  HAL_Init();

 // init();

  //MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  
  // Enable Timer interrupt (TIM7)
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  HAL_TIM_Base_Start(&htim2);
  TIM2->EGR |= TIM_EGR_UG;

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  // compute step sizes from key frequencies
  for (int i = 0; i < 12; ++i) {
    stepSizes[i] = (uint32_t)(4294967296.0 * keyFreqs[i] / 22000.0);
  }

  // create systate mutex
  sysState.mutex = xSemaphoreCreateMutex();

  // create double buffering mutex
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

  // init CAN mailbox semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  // init CAN RX/TX buffers (36 8 byte messages)
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  // init keyPressed to 0xFFFF (no key pressed)
  for(int i = 0; i < 2*CHANNELS; i++)
  {
    sysState.keyPressed[i] = 0xFFFF;
  }

  #ifndef TESTING
  // start key scanning thread
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  512,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  // start screen update thread
  TaskHandle_t displayUpdate = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  512,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdate );	/* Pointer to store the task handle */

  // TODO: check if this stasck size is too much
  // TODO: check task priority
  // start decode CAN message thread
  TaskHandle_t receiveCan = NULL;
  xTaskCreate(
  receiveCanTask,		/* Function that implements the task */
  "receiveCan",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &receiveCan );	/* Pointer to store the task handle */

  TaskHandle_t sendCan = NULL;
  xTaskCreate(
  sendCanTask,		/* Function that implements the task */
  "sendCan",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &sendCan );	/* Pointer to store the task handle */

  TaskHandle_t checkHandshake = NULL;
  xTaskCreate(
  checkHandshakeTask,		/* Function that implements the task */
  "checkHandshake",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &checkHandshake );	/* Pointer to store the task handle */

  TaskHandle_t genBuffer = NULL;
  xTaskCreate(
  genBufferTask,		/* Function that implements the task */
  "genBuffer",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  5,			/* Task priority */
  &genBuffer );	/* Pointer to store the task handle */
  
  // Create the monitor task
  xTaskCreate(TaskMonitor, "Monitor", 2000, NULL, 1, NULL);

  #endif

  // compute sine values for lookup table
  uint32_t phase = 0;
  while(phase < SINE_RESOLUTION)
  {
    sineLookup[phase] = std::round(2048 * std::sin((2*M_PI*phase)/SINE_RESOLUTION));
    phase+=1;
  }

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);

  // disable because we use ADC
  // pinMode(JOYX_PIN, INPUT);
  // pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  setOutMuxBit(HKOW_BIT, HIGH);
  delayMicroseconds(2);
  setOutMuxBit(HKOE_BIT, HIGH);

  // allows east/west signals to stabilise before they are read (IMPORTANT)
  delay(100);

  //Initialise UART
  // Serial.begin(9600);

  // init CAN bus
  CAN_Init(false); // true means it reads it's OWN CAN output
  setCANFilter(0x123,0x7ff);

  #ifndef TESTING
  // bind ISR to CAN RX event
  CAN_RegisterRX_ISR(CAN_RX_ISR);

  // bind ISR to CAN MAILBOX FREE event
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif

  CAN_Start();

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  #ifndef TESTING
  // start RTOS scheduler
  vTaskStartScheduler();
  #endif

  #ifdef TESTING
  Serial.println("========== START TESTING ==========");
  test_sampleISR();
  test_receiveCanTask();
  test_displayUpdateTask();
  //test_sendCanTask();
  test_scanKeysTask();
  test_genBufferTask();
  Serial.println("========== END TESTING ==========");
  //test_checkHandshakeTask();
  #endif
}

void loop() 
{  
}