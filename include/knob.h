#ifndef KNOB_H
#define KNOB_H


// Remember Knob is accessed from a ISR so no MUTEX!
class Knob
{
    public: 
        // constructor
        Knob(int _min, int _max, int _initVal);

        // send voltages to determine knob position
        void updateQuadInputs(int currentA, int currentB);

        // get knob value
        int getValue();

        // set knob value
        int setValue(int _value);

    private:
        int max;
        int min;
        int value = 0;
        int valueNext = 0;

        int lastA = 0;
        int lastB = 0;
        int lastTrans = 0;
};

#endif