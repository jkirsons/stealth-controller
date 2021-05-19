#ifndef COMMANDERCAN_H
#define COMMANDERCAN_H

#include "communication/Commander.h"
#include "canstream.h"

class CommanderCAN : public Commander
{
  public:

    CommanderCAN(CANStream &can, char eol = '\n', bool echo = false);
    CANStream* can_stream = nullptr;  
    
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