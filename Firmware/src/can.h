#ifndef CANDRIVER_H
#define CANDRIVER_H

#include "driver/gpio.h"
#include "driver/can.h"
#include "Print.h"
#include <map>

#include "communication/Commander.h"
#include "canstream.h"

class CANDriver
{
  public:
    CANDriver();
    void init(int tx, int rx, Commander command);
    void transmit();
    void receive();
    float bytesToFloat(uint8_t * bytes);
    uint8_t const * const floatToBytes(float * f);
    double bytesToDouble(uint8_t * bytes);
    uint8_t * getUniqueID();
    u_char getBits(uint32_t value, u_char index);
    CANStream stream;
    Commander command;
    
  private:
};

#endif