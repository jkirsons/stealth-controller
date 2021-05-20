#include "CANStream.h"

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