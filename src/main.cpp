#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <knob.h>
#include <cstring>


// WE ONLY DO SEMAPHORES AND MUTEXES TO ENSURE NOTHING GETS READ WHILST WE ARE STILL WRITING TO IT
// HENCE WHY AN ATOMIC STORE IS IMPORTANT BUT NOT AN ATOMIC READ
// THE ISR MAY INTERRUPT A PROCESS AT ANY POINTS, ALWAYS REMEMBER

// ISRs INTERRUPTS ALL RTOS TASKS

// currentStepSize can be read by the sampling ISR at any point so it must always be in a valid state

// We use atomic stores, in case the variable being written to is read in an ISR, which may be called mid-way through the writing process.
// An atomic load is used when a variable is being read from which may be being written to at the same time (we don't have this case in our code)

#include <ES_CAN.h>

  bool sender = false;
  bool handshakePending = false;

  bool westDetect;
  bool eastDetect;

  int BOARD_ID;
  int ID_RECV = -1;

  int outBits[7] = {0, 0, 0, 1, 1, 1, 1};
  int outBits_new[7] = {0, 0, 0, 0, 0, 0, 0};
  // last two are west and east (they've both been turned on)

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
  const int CHANNELS = 3;

  // CAN message buffers
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

  // knob inits
  Knob volumeKnob = Knob(0, 8, 5);
  Knob octaveKnob = Knob(0, 8, 0);
  Knob waveKnob = Knob(0, 2, 0);

  // signed value from -1
  volatile int joystick = 0;

  // stores the current step sizes that should be played
  volatile uint32_t currentStepSize[CHANNELS];
  // stores the current step sizes that should be played FROM OTHER BOARDS
  volatile uint32_t currentStepSize_EXT[CHANNELS] = {0};

  // TODO: make these memory safe by moving into sysState (described in Lab2)
  // stores the last sent/received CAN message
  uint8_t TX_Message[8] = {0};
  uint8_t RX_Message[8] = {0};

  // frequencies for each key at octave 0
  const int keyFreqs[12] = {65, 69, 73, 78, 82, 87, 93, 98, 104, 110, 116, 123};
  
  // stores the step sizes corresponding with each key at octave 0
  uint32_t stepSizes[12];

  // stores sine wave with SINE_RESOLUTION x-values with amplitude 127 to -127
  const int SINE_RESOLUTION_BITS = 8;
  const int SINE_RESOLUTION = 1 << SINE_RESOLUTION_BITS;  // 2^SINE_RESOLUTION_BITS
  int sineLookup[SINE_RESOLUTION];

  bool octaveOverride = false;

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

void receiveCanTask(void * pvParameters)
{
  int octaveCurrent;
  while(1)
  {
    // blocks until data is available
    // yields the CPU to other tasks in the meantime
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

    uint8_t action = RX_Message[0];
    
    // TODO: bring this back later
    // octave change
    // if(RX_Message[0] == 'O')
    // {
    //   octaveOverride = true;
    //   int8_t octave = (int8_t)RX_Message[1]; // needs to be signed
    //   Serial.println(octave);
    //   octaveCurrent = octaveKnob.getValue();
    //   octaveKnob.setValue(octaveCurrent + (int)octave);
    // }

    // receive ID during handshake
    if(RX_Message[0] == 'I')
    {
      if(handshakePending)
      {
         ID_RECV = RX_Message[1];
      }
    }
    else if(RX_Message[0] == 'E')
    {
      // turn east back on
      outBits[6] = 1; 
    }

    if(!sender)
    {
      uint8_t octave = RX_Message[1];
      uint8_t key = RX_Message[2];
      uint32_t stepSize = stepSizes[key] << octave;

      // key press
      if(RX_Message[0] == 'P')
      {
        for(int i = 0; i < CHANNELS; i++)
        {
          // can only play if we have available channels
          if(currentStepSize_EXT[i] == 0)
          {
            __atomic_store_n(&currentStepSize_EXT[i], stepSize, __ATOMIC_RELAXED);
            break;
          }
        } 
      }

      // key release
      if(RX_Message[0] == 'R')
      {
        for(int i = 0; i < CHANNELS; i++)
        {
          if(currentStepSize_EXT[i] == stepSize)
          {
            __atomic_store_n(&currentStepSize_EXT[i], 0, __ATOMIC_RELAXED);
            break;
          }
        } 
      }
    }
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
  // send a "release" CAN message
  TX_Message[0] = 'I';
  TX_Message[1] = boardID;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

// task to check keys pressed runs every 50ms
void scanKeysTask(void * pvParameters)
{
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // stores latest key presses before being stored globally
  int keyPressedNew[CHANNELS];

  // stores last state of east/west
  // forces a check on startup
  int lastWest = -1;
  int lastEast = -1;

  int octave = 0;
  int lastOctave = 0;

  
  while(1)
  {
    // ensures we sample keyboard only every 50ms
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    // TODO: why can't we just access the actual global?
    // ensures we only access the actual variable once per loop
    uint32_t currentStepSize_Local[CHANNELS] = {0};

    bool westGoesHigh ;

    // init keyPressedNew to -1
    for(int i = 0; i < CHANNELS; i++){ keyPressedNew[i] = -1; }

    // TODO: make this atomic cos joystick is read in ISR
    // top is 130 bottom is 890
    int max = 777;
    int min = 177;
    int joystick_temp = analogRead(JOYY_PIN);
    if(joystick_temp > max)
    {
      joystick_temp = max;
    }
    else if(joystick_temp < min)
    {
      joystick_temp = min;
    }
    // scale from -100 to 100
    joystick_temp = (-joystick_temp + 477) / 3;
    if(joystick_temp < 5 && joystick_temp > -5)
    {
      joystick_temp = 0;
    }

    __atomic_store_n(&joystick, joystick_temp, __ATOMIC_RELAXED);
  
    // get all 32 inputs
    for(int i = 0; i < 8; i++ )
    {
      setRow(i);
      digitalWrite(OUT_PIN, outBits[i]);
      
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
        // only finite key presses at a time
        if(j < CHANNELS)
        {
          // each octave the frequencies are doubled
          currentStepSize_Local[j] = (stepSizes[i] << octaveKnob.getValue()) * (1 + static_cast<float>(joystick)/150);
          keyPressedNew[j] = i;
          j++;
        }
      }
    } 
    
    // TODO: bring this back later
    // if(octaveOverride == false)
    // {
    //   octave = octaveKnob.getValue();
    //   if(octave != lastOctave)
    //   {
    //     // send a +N octave message
    //     TX_Message[0] = 'O';
    //     TX_Message[1] = octave - lastOctave;
    //     xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    //     lastOctave = octave;
    //   }
    // }
    // else
    // {
    //   octaveOverride = false;
    // }
    

    // check CAN inputs for keyboards on left and right
    westDetect = !sysState.inputs[23];
    eastDetect = !sysState.inputs[27];

    Serial.println(BOARD_ID);

    int handshakeDelayStart;
    
    // if handshake inputs change
    if (lastWest != westDetect || lastEast != eastDetect)
    {
      // west went from low to high
      if(lastWest == 0 && westDetect == 1)
      {
        westGoesHigh = true;
      }
      else
      {
        westGoesHigh = false;
      }

      Serial.println("START HANDSHAKE!");

      // left most board
      if (!westDetect)
      {
        // wait long enough to recv messages
        handshakePending = true;

        ID_RECV = -1;


        xSemaphoreGive(sysState.mutex);
        // YIELD TO ANOTHER TASK
        vTaskDelay(pdMS_TO_TICKS(1000));
        // COME BACK WHEN THERES BEEN AN ID MESSAGE

        // take systate mutex
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);

        handshakePending = false;  // Reset flag
        
        // NO board id received...
        if (ID_RECV == -1)
        {
          if(westGoesHigh) // this is just the boards turning on their east after handshake
          {
            continue;
          }
          Serial.println("NO BOARD ID RECEIVED!");
          BOARD_ID = 0;
          sender = false;
        }
        else
        {
          BOARD_ID = ID_RECV;
          sender = true;
        }
        
        octaveKnob.setValue(BOARD_ID);

       
        if(eastDetect)
        {

          // turn east off
          outBits[6] = 0;
          setOutMuxBit(HKOE_BIT, LOW);

          delay(500);
          sendID(BOARD_ID+1);
        }
        else // if you have an easternly board
        {
          // send a "end handshake" CAN message
          TX_Message[0] = 'E';
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        }
      }

      lastWest = westDetect;
      lastEast = eastDetect;
    } 

 
    
    if(sender)
    {
      // check for key releases
      for(int i = 0; i < CHANNELS; i++)
      {
        bool found = false;
        for(int j = 0; j < CHANNELS; j++)
        {
          if(sysState.keyPressed[i] == keyPressedNew[j])
          {
            found = true;
          }
        }

        if(!found)
        {
          // send a "release" CAN message
          TX_Message[0] = 'R';
          TX_Message[1] = octaveKnob.getValue();
          TX_Message[2] = sysState.keyPressed[i];
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        }
      }

      // check for new presses
      for(int i = 0; i < CHANNELS; i++)
      {
        bool found = false;
        for(int j = 0; j < CHANNELS; j++)
        {
          if(sysState.keyPressed[j] == keyPressedNew[i])
          {
            found = true;
          }
        }

        if(!found)
        {
          // send a "press" CAN message
          TX_Message[0] = 'P';
          TX_Message[1] = octaveKnob.getValue();
          TX_Message[2] = keyPressedNew[i];
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        }
      }
    }

    // check knob rotation
    volumeKnob.updateQuadInputs(sysState.inputs[12], sysState.inputs[13]);
    octaveKnob.updateQuadInputs(sysState.inputs[14], sysState.inputs[15]);
    waveKnob.updateQuadInputs(sysState.inputs[16], sysState.inputs[17]);

    // store key presses in sysState
    for(int i = 0; i < CHANNELS; i++)
    {
      sysState.keyPressed[i] = keyPressedNew[i];
    }

    // release systate mutex
    xSemaphoreGive(sysState.mutex);
  
    // atomic write to currentStepSize global
    // if the ISR only receives half of the updated values thats fine, but we can't have half-written elements
    for(int i = 0; i < CHANNELS; i++)
    {
      __atomic_store_n(&currentStepSize[i], currentStepSize_Local[i], __ATOMIC_RELAXED);
    }
  } 
}

// task to update OLED display every 100ms
void displayUpdateTask(void * pvParameters)
{
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // strings used for display
  const char* keyNames [12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  const char* waveTypes[3] = {"Saw", "Sine", "Square"};

  while(1)
  {
    // ensures we update screen every 100ms
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // take systate mutex
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    
    // display notes pressed on current keyboard
    u8g2.setCursor(2,10);
    u8g2.print("Keys: ");

    // Print the keys that are pressed
    for (int i = 0; i < CHANNELS; i++) {
      if (sysState.keyPressed[i] != -1) {
        u8g2.print(keyNames[sysState.keyPressed[i]]);
        u8g2.print(" ");
      }
    }

    // Determine the width of the display
    int displayWidth = u8g2.getDisplayWidth();
    int textWidth = u8g2.getStrWidth("00"); // RECV and SEND have same width
    u8g2.setCursor(displayWidth - textWidth, 10);
    u8g2.print(BOARD_ID);

    // display octave of current keyboard
    u8g2.setCursor(2,20);
    u8g2.print("Oct: ");
    u8g2.print(octaveKnob.getValue());

    // display wave type of current keyboard
    u8g2.print("   Wave: ");
    u8g2.print(waveTypes[waveKnob.getValue()]);

    // display volume of curren keyboard
    u8g2.setCursor(2,30);
    u8g2.print("Vol: ");
    u8g2.print(volumeKnob.getValue());

    // display last received CAN message
    u8g2.setCursor(63,30);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

    if(handshakePending)
    {
      textWidth = u8g2.getStrWidth("HAND");
      u8g2.setCursor(displayWidth - textWidth, 10);
      u8g2.print("HAND");
    }

    textWidth = u8g2.getStrWidth("HAND");
    u8g2.setCursor(displayWidth - textWidth, 30);
    u8g2.print(westDetect);
    u8g2.print(eastDetect);

    // push screen buffer to display
    u8g2.sendBuffer();

    // toggle lED
    digitalToggle(LED_BUILTIN);

    // release systate mutex
    xSemaphoreGive(sysState.mutex);
  } 
}

// sets speaker voltage 22,000 times per sec
// TODO: read systate and currentStepSize atomically
void sampleISR()
{
  // TODO: make the ISR not be attached to a sender
  // sender doesn't play notes
  if(sender) return;

  // stores phase for each channel wave separately
  static uint32_t phaseAcc[CHANNELS*2] = {0};

  // stores current voltage of speaker
  int32_t Vout = 0;
  
  int waveType = waveKnob.getValue();
  
  for(int i = 0; i < CHANNELS*2; i++)
  {
    if(i < CHANNELS)
    {
      phaseAcc[i] += currentStepSize[i];
    }
    else
    {
      phaseAcc[i] += currentStepSize_EXT[i - CHANNELS];
    }
    
    // sawtooth wave
    if(waveType == 0)
    {
      Vout += (phaseAcc[i] >> 24) - 128;
    }
    // sine wave
    else if(waveType == 1)
    {
      int angle = (phaseAcc[i] >> (32 - SINE_RESOLUTION_BITS)) & 0xFF; 
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

  // send volatage from 0-255 to speaker
  Vout = constrain(Vout + 128, 0, 255);   
  analogWrite(OUTR_PIN, Vout);
}

// scan TX mailbox and wait until a mailbox is available
void sendCanTask (void * pvParameters) {
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
  // compute step sizes from key frequencies
  for (int i = 0; i < 12; ++i) {
    stepSizes[i] = (uint32_t)(4294967296.0 * keyFreqs[i] / 22000.0);
  }

  // create systate mutex
  sysState.mutex = xSemaphoreCreateMutex();

  // init CAN mailbox semaphore
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  // init CAN RX/TX buffers (36 8 byte messages)
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  // init keyPressed to all -1
  for(int i = 0; i < CHANNELS; i++)
  {
    sysState.keyPressed[i] = -1;
  }

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
  
  // compute sine values for lookup table
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

  setOutMuxBit(HKOW_BIT, HIGH);
  delayMicroseconds(2);
  setOutMuxBit(HKOE_BIT, HIGH);

  // allows east/west signals to stabilise before they are read (IMPORTANT)
  delay(1000);
  

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // initialise sampling interrupt (22,000 times a sec)
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();


  // init CAN bus
  CAN_Init(false); // true means it reads it's OWN CAN output
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