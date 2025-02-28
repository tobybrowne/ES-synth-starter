#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <math.h>

// WE ONLY DO SEMAPHORES AND MUTEXES TO ENSURE NOTHING GETS READ WHILST WE ARE STILL WRITING TO IT
// HENCE WHY AN ATOMIC STORE IS IMPORTANT BUT NOT AN ATOMIC READ
// THE 

// #include <ES_CAN.h>

// Remember Knob is accessed from a ISR so no MUTEX!
// is this right to not use a mutex (is atomic ops sufficient?)
class Knob
{
  public: 
    int max;
    int min;
    int value = 0;
    int valueNext = 0;

    int lastA = 0;
    int lastB = 0;
    int lastTrans = 0;
    
    Knob(int _min, int _max, int _initVal)
    {
      max = _max;
      min = _min;
      value = _initVal;
      valueNext = value;
    }

    void updateQuadInputs(int currentA, int currentB)
    {
      int current_state = (currentA << 1) | currentB; // Combine A and B into a 2-bit state
      int prev_state = (lastA << 1) | lastB; // Combine previous A and B into a 2-bit state
      
      int trans = 0;

      switch (prev_state)
      {
        case 0b00:
          if (current_state == 0b00);
          else if (current_state == 0b01) trans--;
          else if (current_state == 0b10);
          else if (current_state == 0b11) trans = lastTrans;
          break;
        case 0b01:
          if (current_state == 0b00) trans++;
          else if (current_state == 0b01);
          else if (current_state == 0b10) trans = lastTrans;
          else if (current_state == 0b11);
          break;
        case 0b10:
          if (current_state == 0b00) trans = lastTrans;
          else if (current_state == 0b01);
          else if (current_state == 0b10);
          else if (current_state == 0b11) trans++;
          break;
        case 0b11:
          if (current_state == 0b00) trans = lastTrans;
          else if (current_state == 0b01);
          else if (current_state == 0b10) trans--;
          else if (current_state == 0b11);
          break;
      }

      lastA = currentA;
      lastB = currentB;

      valueNext += trans;

      // limit knob between 0 and 8
      if(valueNext > max)
      {
        valueNext = max;
      }
      else if(valueNext < min)
      {
        valueNext = min;
      }

      if(trans != 0)
      {
        lastTrans = trans;
      } 

      // atomic store, because knob value can be accessed from ISR
      __atomic_store_n(&value, valueNext, __ATOMIC_RELAXED);
    }

    // atomic load but no mutex acquired (so can be called from ISR)
    int getValue()
    {
      return __atomic_load_n(&value, __ATOMIC_RELAXED);
    }
};

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

  // store inputs state
  struct
  {
    std::bitset<32> inputs; 
    int keyPressed[CHANNELS];
    SemaphoreHandle_t mutex; 
  } sysState;

  Knob volumeKnob = Knob(0, 8, 5);
  Knob octaveKnob = Knob(0, 8, 0);
  Knob waveKnob = Knob(0, 1, 0);

  const char* waveTypes[2] = {"Sawtooth", "Sine"};

  // stores the current step size that should be played
  volatile uint32_t currentStepSize[3];

  // CAN OUT message
  // TODO: make this just a local inside scankeys (stop displaying on screen)
  uint8_t TX_Message[8] = {0};

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

// task to check keys pressed every 50ms
void scanKeysTask(void * pvParameters)
{
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int lastKeyPressed = -1;
  
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
      sysState.keyPressed[i] = -1;
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
          // currentStepSize_local = stepSizes[i];
          sysState.keyPressed[j] = i;
          j++;
        }
        
      }
    } 

    // // check if key changed
    // if(keyPressed != lastKeyPressed)
    // {
    //   // key release to nothing
    //   if(keyPressed == -1)
    //   {
    //     TX_Message[0] = 'R';
    //     TX_Message[1] = octaveKnob.getValue();
    //     TX_Message[2] = lastKeyPressed;
    //   }
    //   // new key pressed
    //   else
    //   {
    //     TX_Message[0] = 'P';
    //     TX_Message[1] = octaveKnob.getValue();
    //     TX_Message[2] = keyPressed;
    //   }

    //   // send message down CAN bus
    //   //CAN_TX(0x123, TX_Message);
    // }

   // lastKeyPressed = keyPressed;
    //sysState.keyPressed = keyPressed;
      
    // check knob rotation (knob 3)
    volumeKnob.updateQuadInputs(sysState.inputs[12], sysState.inputs[13]);
    octaveKnob.updateQuadInputs(sysState.inputs[14], sysState.inputs[15]);
    waveKnob.updateQuadInputs(sysState.inputs[16], sysState.inputs[17]);

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
    u8g2.print("Key Pressed: ");  // write something to the internal memory
    
    for(int i = 0; i < CHANNELS; i++)
    {
      if(sysState.keyPressed[i] != -1)
      {
        u8g2.print(keyNames[sysState.keyPressed[i]]);
        u8g2.print(" ");
      }
    }

    u8g2.setCursor(2,20);
    u8g2.print("Octave: ");
    u8g2.print(octaveKnob.getValue());
    u8g2.print("  Wave: ");
    u8g2.print(waveTypes[waveKnob.getValue()]);

    u8g2.setCursor(2,30);
    u8g2.print("Volume: ");
    u8g2.print(volumeKnob.getValue());

    // scan for CAN input
    // uint32_t ID;
    // uint8_t RX_Message[8] = {0};
    // while (CAN_CheckRXLevel())
    //   CAN_RX(ID, RX_Message);

    // u8g2.setCursor(66,30);
    // u8g2.print((char) RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    // release systate mutex
    xSemaphoreGive(sysState.mutex);
  } 
}

int8_t compute_sine(uint32_t phase)
{
  // converts value from 0->2^32 to 0->SIN_RESOLUTION for use with the lookup table
  // 
  //int angle = (phase * SINE_RESOLUTION) >> 32;
  // int angle = (phase >> 24) & 0xFF;
  // Serial.println(phase);
  //int angle = std::round(static_cast<double>(phase) * SINE_RESOLUTION / 4294967295.0);
  // Serial.println(angle);
  // int angle = (phase * SINE_RESOLUTION) >> 32;
  
  // hard programmed for 256 sine lookup table
  int angle = (phase >> (32 - SINE_RESOLUTION_BITS)) & 0xFF;  // Shift phase and mask to get a value between 0 and 255

  return sineLookup[angle];
}


// ISR that runs 22,000 times per sec
// TODO: read systate and csurrentStepSize atomically
void sampleISR()
{
  // will be filled with 0s
  static uint32_t phaseAcc[CHANNELS] = {0}; // this declaration only happens once
  int32_t Vout = 0; // declaration happens everytime
  
  for(int i = 0; i < CHANNELS; i++)
  {
    phaseAcc[i] += currentStepSize[i];

    // sine wave
    if(waveKnob.getValue())
    {
      Vout += compute_sine(phaseAcc[i]);
      // Vout += (phaseAcc[i] >> 24) - 128;
    }
    // sawtooth wave
    else
    {
      Vout += (phaseAcc[i] >> 24) - 128;
    }
  }

  // log-taper volume control
  Vout = Vout >> (8 - volumeKnob.getValue());
  Vout = constrain(Vout + 128, 0, 255);   
  analogWrite(OUTR_PIN, Vout);
}

void setup() {
  // put your setup code here, to run once:

  // compute step sizes from key frequencies
  for (int i = 0; i < 12; ++i) {
    stepSizes[i] = (uint32_t)(4294967296.0 * keyFreqs[i] / 22000.0);
  }

  // create systate mutex
  sysState.mutex = xSemaphoreCreateMutex();

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

  // initialise sampling interrupt (22,000 times a sec)
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // init CAN bus
  // CAN_Init(true); // true means it reads it's OWN CAN output
  // setCANFilter(0x123,0x7ff);
  // CAN_Start();

  // start RTOS scheduler
  vTaskStartScheduler();
}

void loop() 
{  
}