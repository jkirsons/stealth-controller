#include "canstream.h"


size_t CANStream::print(int n, int base) {
    dataType = dt::int_val;
    int_value = n;
    return sizeof(n);
}

size_t CANStream::println(int n, int base) {
    return print(n, base);
}

size_t CANStream::print(double n, int digits) {
    dataType = dt::double_val;
    double_value = n;
    return sizeof(n);
}

size_t CANStream::println(double n, int digits) {
    return print(n, digits);
}

size_t CANStream::print(const Printable& x) {
    return 0;
}

size_t CANStream::println(const Printable& x) {
    return 0;
}

size_t CANStream::print(const __FlashStringHelper *ifsh) {
    return 0;
}

size_t CANStream::println(const __FlashStringHelper *ifsh) {
    return 0;
}

size_t CANStream::print(char c) {
    dataType = dt::char_val;
    char_value = c;
    return sizeof(c);
}

size_t CANStream::println(char c) {
    return print(c);
}

int CANStream::available(void) { 
    return (value_position < strlen(value_buffer)); 
}

int CANStream::read(void) { 
    if(value_position < strlen(value_buffer)) {
        return value_buffer[value_position++];
    }
    return 0; 
}

int CANStream::peek(void) { return 0; }
void CANStream::flush(void) {}

size_t CANStream::readBytes(char *buffer, size_t length) { return 0; }
String CANStream::readString() { return String(""); }

size_t CANStream::write(uint8_t) { return 0; }
size_t CANStream::write(const uint8_t *buffer, size_t size) { return 0; }