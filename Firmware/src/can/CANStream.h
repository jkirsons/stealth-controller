#ifndef CANSTREAM_H
#define CANSTREAM_H

#include "Stream.h"
#include "communication/Commander.h"

class CANStream: public Stream
{
public:
    int available();
    int read();
    int peek();
    void flush();

    size_t readBytes(char *buffer, size_t length);
    String readString();

    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);

    enum dt { double_val, int_val, char_val, none } dataType = CANStream::dt::none;
    double double_value;
    int int_value;
    char char_value;

    char value_buffer [100];
    int value_position = 100;
};

#endif