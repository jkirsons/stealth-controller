#ifndef CANDSTREAM_H
#define CANSTREAM_H

#include "Stream.h"
#include "communication/Commander.h"

class CANStream: public Stream
{
public:

    size_t print(int, int = DEC);
    size_t println(int, int = DEC);
    size_t print(double, int = 2);
    size_t println(double, int = 2);
    size_t print(const Printable&);
    size_t println(const Printable&);
    size_t print(const __FlashStringHelper *);
    size_t println(const __FlashStringHelper *);
    size_t print(char);
    size_t println(char);

    int available();
    int read();
    int peek();
    void flush();

    size_t readBytes(char *buffer, size_t length);
    String readString();

    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);

    enum dt { double_val, int_val, char_val } dataType;
    double double_value;
    int int_value;
    char char_value;

    char value_buffer [100];
    int value_position = 100;
};

#endif