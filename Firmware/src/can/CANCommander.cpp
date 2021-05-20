#include "CANCommander.h"

CANCommander::CANCommander(CANStream &can, char eol, bool echo)
{
  can_stream = &can;
  com_port = &can;
  this->eol = eol;
  this->echo = echo;
}

// Intercept the print commands, and store the original data that was sent.
void CANCommander::print(const int number){
  if(!can_stream) return;
  can_stream->dataType = can_stream->int_val;
  can_stream->int_value = number;
}
void CANCommander::print(const float number){
  if(!can_stream) return;
  can_stream->dataType = can_stream->double_val;
  can_stream->double_value = number;
}
void CANCommander::print(const char* message){
  if(!can_stream) return;
  //can_port->print(message);
} 
void CANCommander::print(const __FlashStringHelper *message){
  if(!can_stream) return;
    
}
void CANCommander::print(const char message){
  if(!can_stream) return;
  can_stream->dataType = can_stream->char_val;
  can_stream->char_value = message;
}

void CANCommander::println(const int number){
  print(number);
}
void CANCommander::println(const float number){
  print((float)number);
}
void CANCommander::println(const char* message){
  print(message);
} 
void CANCommander::println(const __FlashStringHelper *message){
  print(message);
}
void CANCommander::println(const char message){
  print(message);
}
