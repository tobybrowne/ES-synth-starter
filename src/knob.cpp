#include "../include/knob.h"

// constructor
Knob::Knob(int _min, int _max, int _initVal)
{
  max = _max;
  min = _min;
  value = _initVal;
  valueNext = value;
}

// send voltages to determine knob position
void Knob::updateQuadInputs(int currentA, int currentB)
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

  // limit knob between min and max
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

// get knob value
int Knob::getValue()
{
  return __atomic_load_n(&value, __ATOMIC_RELAXED);
}
