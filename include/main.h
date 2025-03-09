#ifndef MAIN_H
#define MAIN_H

#include <bitset>
#include <STM32FreeRTOS.h>

// how many keys can be pressed together
const int CHANNELS = 12;

struct SysState
{
    std::bitset<32> inputs;
    uint16_t keyPressed[CHANNELS*2]; // stores octave-index pairs for all keys pressed 
    bool sender = false;
    bool handshakePending = false;

    bool reverb = true;
    bool stereo = false;

    bool rightBoard = false;
    int BOARD_ID;

    int ID_RECV = -1;

    bool octaveOverride = false;

    SemaphoreHandle_t mutex; 
};

#endif