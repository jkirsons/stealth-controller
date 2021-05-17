#ifndef CANDRIVER_H
#define CANDRIVER_H

#include "driver/gpio.h"
#include "driver/can.h"
#include <map>

#include "communication/Commander.h"
#include "canstream.h"

#include "can_api.h"

class CANDriver
{
  public:
    CANDriver();
    void init(int tx, int rx, Commander command);
    void transmit();
    void receive();
    int32_t bytesToInt(uint8_t * bytes);
    float bytesToFloat(uint8_t * bytes);
    double bytesToDouble(uint8_t * bytes);
    uint8_t const * const floatToBytes(float * f);
    uint8_t getBits(uint32_t value, uint8_t index);
    
    CANStream stream;
    Commander command;
    uint32_t identifier;
    
  private:
};

#endif