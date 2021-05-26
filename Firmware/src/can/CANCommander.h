#ifndef CANCOMMANDER_H
#define CANCOMMANDER_H

#include "communication/Commander.h"
#include "CANDriver.h"

class CANCommander : public Commander
{
  public:

    CANCommander(CANDriver &can, char eol = '\n', bool echo = false);
    void runWithCAN();
    CANDriver* can_driver = nullptr;  
    
  private:
    void print(const float number) override;
    void print(const int number) override;
    void print(const char* message) override;
    void print(const __FlashStringHelper *message) override;
    void print(const char message) override;
    void println(const float number) override;
    void println(const int number) override;
    void println(const char* message) override;
    void println(const __FlashStringHelper *message) override;
    void println(const char message) override;

};

#endif