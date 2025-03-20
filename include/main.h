#ifndef MAIN_H
#define MAIN_H

#include <bitset>
#include <STM32FreeRTOS.h>

// how many keys can be pressed together
const int CHANNELS = 1;

// stores device state
struct SysState
{
    std::bitset<32> inputs;
    uint16_t keyPressed[CHANNELS*2]; // stores octave-index pairs for all keys pressed 
    bool sender = false;
    bool handshakePending = false;

    bool reverb = false;
    bool stereo = false;
    int octave = 0;
    int volume = 0;
    int waveType = 0;

    bool rightBoard = false;
    int BOARD_ID;

    int ID_RECV = -1;

    bool octaveOverride = false;

    SemaphoreHandle_t mutex; 
};


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

#endif