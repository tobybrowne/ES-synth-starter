#include <Arduino.h>
#include <U8g2lib.h>
#include "frames.c"  // Include animation frames

// Constants
const uint32_t interval = 100; // Display update interval

// Pin definitions
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
    digitalWrite(REN_PIN, LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN, value);
    digitalWrite(REN_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN, LOW);
}

// Animation frames
extern const unsigned char frame_0_bmp[] PROGMEM;
extern const unsigned char frame_1_bmp[] PROGMEM;
extern const unsigned char frame_2_bmp[] PROGMEM;
extern const unsigned char frame_3_bmp[] PROGMEM;
extern const unsigned char frame_4_bmp[] PROGMEM;
extern const unsigned char frame_5_bmp[] PROGMEM;
extern const unsigned char frame_6_bmp[] PROGMEM;
extern const unsigned char frame_7_bmp[] PROGMEM;
extern const unsigned char frame_8_bmp[] PROGMEM;
extern const unsigned char frame_9_bmp[] PROGMEM;
extern const unsigned char frame_10_bmp[] PROGMEM;

const unsigned char* frames[] = {
    frame_0_bmp, frame_1_bmp, frame_2_bmp, frame_3_bmp, frame_4_bmp,
    frame_5_bmp, frame_6_bmp, frame_7_bmp, frame_8_bmp, frame_9_bmp,
    frame_10_bmp
};

const int numFrames = 11; // Update this if you have more frames
int currentFrame = 0;

void setup() {
    // Set pin directions
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

    // Initialize display
    setOutMuxBit(DRST_BIT, LOW);  
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);

    // Initialize UART
    Serial.begin(9600);
    Serial.println("OLED Animation Test");
}

void loop() {
    static uint32_t next = millis();

    while (millis() < next);  // Wait for next frame interval
    next += interval;

    // Display animation frame
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 32, frames[currentFrame]);  // Draw the current frame
    u8g2.sendBuffer();

    currentFrame = (currentFrame + 1) % numFrames;  // Loop through frames

    // Toggle LED (debugging)
    digitalToggle(LED_BUILTIN);
}
