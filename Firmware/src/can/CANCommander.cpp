#include "CANCommander.h"

CANCommander::CANCommander(CANDriver &can, char eol, bool echo)
{
  can_driver = &can;
  can_driver->command = *this;
  com_port = &can.stream;
  this->eol = eol;
  this->echo = echo;
}

void CANCommander::runWithCAN(void) {
  can_driver->receive();
  run();
  can_driver->transmit();
}

// Intercept the print commands, and store the original data that was sent.
void CANCommander::print(const int number){
  if(!can_driver) return;
  can_driver->stream.dataType = can_driver->stream.int_val;
  can_driver->stream.int_value = number;
}
void CANCommander::print(const float number){
  if(!can_driver) return;
  can_driver->stream.dataType = can_driver->stream.double_val;
  can_driver->stream.double_value = number;
}
void CANCommander::print(const char* message){
  if(!can_driver) return;
  //can_port->print(message);
} 
void CANCommander::print(const __FlashStringHelper *message){
  if(!can_driver) return;
    
}
void CANCommander::print(const char message){
  if(!can_driver) return;
  can_driver->stream.dataType = can_driver->stream.char_val;
  can_driver->stream.char_value = message;
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
