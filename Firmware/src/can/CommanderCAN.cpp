#include "CommanderCAN.h"

CommanderCAN::CommanderCAN(CANStream &can, char eol, bool echo)
{
  can_stream = &can;
  com_port = &can;
  this->eol = eol;
  this->echo = echo;
}

// Intercept the print commands, and store the original data that was sent.
void CommanderCAN::print(const int number){
  if(!can_stream) return;
  can_stream->dataType = can_stream->int_val;
  can_stream->int_value = number;
}
void CommanderCAN::print(const float number){
  if(!can_stream) return;
  can_stream->dataType = can_stream->double_val;
  can_stream->double_value = number;
}
void CommanderCAN::print(const char* message){
  if(!can_stream) return;
  //can_port->print(message);
} 
void CommanderCAN::print(const __FlashStringHelper *message){
  if(!can_stream) return;
    
}
void CommanderCAN::print(const char message){
  if(!can_stream) return;
  can_stream->dataType = can_stream->char_val;
  can_stream->char_value = message;
}

void CommanderCAN::println(const int number){
  print(number);
}
void CommanderCAN::println(const float number){
  print((float)number);
}
void CommanderCAN::println(const char* message){
  print(message);
} 
void CommanderCAN::println(const __FlashStringHelper *message){
  print(message);
}
void CommanderCAN::println(const char message){
  print(message);
}
