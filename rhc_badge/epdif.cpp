#include "epdif.h"
#include <SPI.h>
EpdIf::EpdIf() {
}

EpdIf::~EpdIf() {
}

void EpdIf::DigitalWrite(int pin, int value) {
    digitalWrite(pin, value);
}

int EpdIf::DigitalRead(int pin) {
    return digitalRead(pin);
}

void EpdIf::DelayMs(unsigned int delaytime) {
    delay(delaytime);
}

void EpdIf::SpiTransfer(unsigned char data) {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(data);
    digitalWrite(CS_PIN, HIGH);
}

int EpdIf::IfInit(void) {
    pinMode(CS_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    pinMode(DC_PIN, OUTPUT);
    pinMode(BUSY_PIN, INPUT);

    pinMode(CLK_PIN, OUTPUT);
    pinMode(DIN_PIN, OUTPUT);

    DigitalWrite(CS_PIN, HIGH);
    DigitalWrite(RST_PIN, HIGH);
    DigitalWrite(DC_PIN, LOW);

    DigitalWrite(CLK_PIN, LOW);
    DigitalWrite(DIN_PIN, LOW);
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    return 0;
}