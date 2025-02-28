// #define SENDER

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <knob.h>

// WE ONLY DO SEMAPHORES AND MUTEXES TO ENSURE NOTHING GETS READ WHILST WE ARE STILL WRITING TO IT
// HENCE WHY AN ATOMIC STORE IS IMPORTANT BUT NOT AN ATOMIC READ
// THE ISR MAY INTERRUPT A PROCESS AT ANY POINTS, ALWAYS REMEMBER

#include <ES_CAN.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

  // how many keys can be pressed together
  const int CHANNELS = 4;

  // CAN RX msg buffer
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;

  // controls acccess to the CAN mailboxes (hardware has 3)
  SemaphoreHandle_t CAN_TX_Semaphore;

  // store inputs state
  struct
  {
    std::bitset<32> inputs; 
    int keyPressed[CHANNELS];
    SemaphoreHandle_t mutex; 
  } sysState;

  Knob volumeKnob = Knob(0, 8, 5);
  Knob octaveKnob = Knob(0, 8, 0);
  Knob waveKnob = Knob(0, 2, 0);

  const char* waveTypes[3] = {"Saw", "Sine", "Square"};

  // stores the current step size that should be played
  volatile uint32_t currentStepSize[CHANNELS];

  // CAN OUT message
  // TODO: make these memory safe by moving into sysState (described in Lab2)
  uint8_t TX_Message[8] = {0};
  uint8_t RX_Message[8] = {0};

  // generate step sizes for each key's note
  // frequencies for octave 0
  const int keyFreqs[12] = {65, 69, 73, 78, 82, 87, 93, 98, 104, 110, 116, 123};
  const char* keyNames [12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  uint32_t stepSizes[12];

  // stores sine wave with 256 x-values with amplitude 127 to -127
  // surely sine resolution above 256 makes no sense
  const int SINE_RESOLUTION_BITS = 8;
  const int SINE_RESOLUTION = 1 << SINE_RESOLUTION_BITS;  // 2^SINE_RESOLUTION_BITS
  int sineLookup[SINE_RESOLUTION];

//Display driver objects
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// define timer object
HardwareTimer sampleTimer(TIM1);

//Function to set outputs using key matrix
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

// task to decode CAN messages in the RX buffer
void decodeTask(void * pvParameters)
{
  while(1)
  {
    // blocks until data is available
    // yields the CPU to other tasks in the meantime
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    #ifndef SENDER
      uint8_t action = RX_Message[0];
      uint8_t octave = RX_Message[0];
      uint8_t key = RX_Message[0];
      uint32_t stepSize = stepSizes[key] << octave;
      
      // key press
      if(RX_Message[0] == 'P')
      {
        for(int i = 0; i < CHANNELS; i++)
        {
          // can only play if we have available channels
          if(currentStepSize[i] == 0)
          {
            currentStepSize[i] = stepSize;
          }
        } 
      }

      // key release
      if(RX_Message[0] == 'R')
      {
        for(int i = 0; i < CHANNELS; i++)
        {
          if(currentStepSize[i] == stepSize)
          {
            currentStepSize[i] = 0;
          }
        } 
      }
      
    #endif
  }
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// task to check keys pressed every 50ms
void scanKeysTask(void * pvParameters)
{
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int lastKeyPressed = -1;
  int keyPressed_local[CHANNELS];
  
  while(1)
  {
    // ensures we sample keyboard only every 50ms
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    // ensures we only access the actual variable once per loop

    uint32_t currentStepSize_local[CHANNELS] = {0};
    for(int i = 0; i < CHANNELS; i++)
    {
      keyPressed_local[i] = -1;
    }
  
    // get all 32 inputs
    for(int i = 0; i < 8; i++ )
    {
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> columns = readCols();
  
      int start_id = i * 4;
      for(int j = 0; j < 4; j++)
      {
        sysState.inputs[start_id + j] = columns[j];
      }  
    }

    // check keyboard presses
    int j = 0;
    for(int i = 0; i < 12; i++)
    {
      if(sysState.inputs[i] == 0) // if key pressed...
      {
        // only three keys at a time
        if(j < CHANNELS)
        {
          // each octave the frequencies are doubled
          currentStepSize_local[j] = stepSizes[i] << octaveKnob.getValue();
          keyPressed_local[j] = i;
          j++;
        }
        
      }
    } 

    #ifdef SENDER
      // check for releases
      for(int i = 0; i < CHANNELS; i++)
      {
        bool found = false;
        // sysState
        for(int j = 0; j < CHANNELS; j++)
        {
          // keyPressed_local
          if(sysState.keyPressed[i] == keyPressed_local[j])
          {
            found = true;
          }
        }

        if(!found)
        {
          TX_Message[0] = 'R';
          TX_Message[1] = octaveKnob.getValue();
          TX_Message[2] = sysState.keyPressed[i];
          xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
          // CAN_TX(0x123, TX_Message);
        }
      }

      // check for new presses
      for(int i = 0; i < CHANNELS; i++)
      {
        bool found = false;
        // keyPressed_local
        for(int j = 0; j < CHANNELS; j++)
        {
          // sysState
          if(sysState.keyPressed[j] == keyPressed_local[i])
          {
            found = true;
          }
        }

        if(!found)
        {
          TX_Message[0] = 'P';
          TX_Message[1] = octaveKnob.getValue();
          TX_Message[2] = keyPressed_local[i];
          xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
          // CAN_TX(0x123, TX_Message);
        }
      }
    #endif
      
    // check knob rotation (knob 3)
    volumeKnob.updateQuadInputs(sysState.inputs[12], sysState.inputs[13]);
    octaveKnob.updateQuadInputs(sysState.inputs[14], sysState.inputs[15]);
    waveKnob.updateQuadInputs(sysState.inputs[16], sysState.inputs[17]);

    // deffo a better way of doing this
    for(int i = 0; i < CHANNELS; i++)
    {
      sysState.keyPressed[i] = keyPressed_local[i];
    }

    // release systate mutex
    xSemaphoreGive(sysState.mutex);
  
    // atomic write to currentStepSize global
    // if this process is interrupted its not a big deal right?
    for(int i = 0; i < CHANNELS; i++)
    {
      __atomic_store_n(&currentStepSize[i], currentStepSize_local[i], __ATOMIC_RELAXED);
    }
  } 
}



// task to update OLED display every 100ms
void displayUpdateTask(void * pvParameters)
{
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    // ensures we update screen every 100ms
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    
    u8g2.setCursor(2,10);
    u8g2.print("Keys: ");  // write something to the internal memory
    
    for(int i = 0; i < CHANNELS; i++)
    {
      if(sysState.keyPressed[i] != -1)
      {
        u8g2.print(keyNames[sysState.keyPressed[i]]);
        u8g2.print(" ");
      }
    }

    u8g2.setCursor(2,20);
    u8g2.print("Oct: ");
    u8g2.print(octaveKnob.getValue());
    u8g2.print("   Wave: ");
    u8g2.print(waveTypes[waveKnob.getValue()]);

    u8g2.setCursor(2,30);
    u8g2.print("Vol: ");
    u8g2.print(volumeKnob.getValue());

    // display last received CAN message
    u8g2.setCursor(66,30);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    // release systate mutex
    xSemaphoreGive(sysState.mutex);
  } 
}


// ISR that runs 22,000 times per sec
// TODO: read systate and csurrentStepSize atomically
void sampleISR()
{
  // will be filled with 0s
  static uint32_t phaseAcc[CHANNELS] = {0}; // this declaration only happens once
  int32_t Vout = 0; // declaration happens everytime
  int waveType = waveKnob.getValue();
  
  for(int i = 0; i < CHANNELS; i++)
  {
    phaseAcc[i] += currentStepSize[i];

    // sawtooth wave
    if(waveType == 0)
    {
      Vout += (phaseAcc[i] >> 24) - 128;
    }
    // sine wave
    else if(waveType == 1)
    {
      int angle = (phaseAcc[i] >> (32 - SINE_RESOLUTION_BITS)) & 0xFF;  // Shift phase and mask to get a value between 0 and 255
      Vout += sineLookup[angle];
    }
    // square wave
    else if(waveType == 2)
    {
      int threshold = (phaseAcc[i] > (1 << 31));
      Vout += (threshold * 256) - 128;
    }
  }

  // log-taper volume control
  Vout = Vout >> (8 - volumeKnob.getValue());
  Vout = constrain(Vout + 128, 0, 255);   
  analogWrite(OUTR_PIN, Vout);
}

// scan TX mailbox and wait until a mailbox is available
void canSendTask (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

// frees CAN semaphore when mailbox is available
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void setup() {
  // put your setup code here, to run once:

  // compute step sizes from key frequencies
  for (int i = 0; i < 12; ++i) {
    stepSizes[i] = (uint32_t)(4294967296.0 * keyFreqs[i] / 22000.0);
  }

  // create systate mutex
  sysState.mutex = xSemaphoreCreateMutex();

  // init CAN mailbox semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  // init CAN RX buffer (36 8 byte messages)
  msgInQ = xQueueCreate(36,8);
  // init CAN TX buffer (36 8 byte messages)
  msgOutQ = xQueueCreate(36,8);

  // start key scanning thread
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  // start screen update thread
  TaskHandle_t displayUpdate = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdate );	/* Pointer to store the task handle */

  // TODO: check if this stasck size is too much
  // TODO: check task priority
  // start decode CAN message thread
  TaskHandle_t decode = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &decode );	/* Pointer to store the task handle */

  TaskHandle_t canSend = NULL;
  xTaskCreate(
  canSendTask,		/* Function that implements the task */
  "canSend",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &canSend );	/* Pointer to store the task handle */
  
  // compute 256 sine values
  uint32_t phase = 0;
  while(phase < SINE_RESOLUTION)
  {
    sineLookup[phase] = std::round(127 * std::sin((2*M_PI*phase)/SINE_RESOLUTION));
    phase++;
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
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // only receivers should play their notes
  #ifndef SENDER
    // initialise sampling interrupt (22,000 times a sec)
    sampleTimer.setOverflow(22000, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();
  #endif

  // init CAN bus
  CAN_Init(true); // true means it reads it's OWN CAN output
  setCANFilter(0x123,0x7ff);

  // bind ISR to CAN RX event
  CAN_RegisterRX_ISR(CAN_RX_ISR);

  // bind ISR to CAN MAILBOX FREE event
  CAN_RegisterTX_ISR(CAN_TX_ISR);

  CAN_Start();

  // start RTOS scheduler
  vTaskStartScheduler();
}

void loop() 
{  
}