#include "can.h"
/*
 * CAN HAT:
 * https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/
 * https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/#using-can-bus-shiled-with-jetson-nano
 * 
 * sudo ip link set can0 up type can bitrate 1000000 fd off
 * cansend can0 00012111#AABBCCDD
 * 
 * sudo ip link set can0 up type can help
 */

uint32_t bitMask[] = { 0b1111,    //BusID
                        0b1,      //Operation
                        0b111,    //DataType
                        0b1111,   //Global
                        0b1111,   //Command
                        0b1111 }; //SubCommand;

int shr[] = { 16, 15, 12, 8, 4, 0 }; // shift right by bits  

// Map identifier bit segments to commander codes.  Structures are generated here:
// https://docs.google.com/spreadsheets/d/10lNGwA5vVTEzVzacbnZFmxrxztgwlqkmvYGRFUPO5Kk/edit#gid=0
std::map<int,char> globals { { 5, '?' }, { 6, '@' }, { 7, '#' }, { 0, 'A' }, { 1, 'B' }, { 2, 'C' }, { 3, 'D' }, { 4, 'E' }, { 10, 'M' }, { 11, 'N' }, { 12, 'O' }, { 13, 'P' }, { 14, 'Q' }, { 15, 'R' } };
std::map<int,char> commands { { 1, 'E' }, { 2, 'L' }, { 3, 'C' }, { 4, 'T' }, { 5, 'S' }, { 6, 'M' }, { 7, 'R' }, { 8, 'D' }, { 9, 'Q' }, { 10, 'V' }, { 11, 'A' } };
std::map<std::pair<int, int>, char> subCommands { { { 2, 1 }, 'C' }, { { 2, 2 }, 'U' }, { { 2, 3 }, 'V' }, { { 3, 1 }, 'D' }, { { 5, 1 }, 'M' }, { { 5, 2 }, 'E' }, { { 6, 1 }, 'D' }, { { 6, 2 }, 'C' }, { { 6, 3 }, 'G' }, { { 6, 4 }, 'S' }, { { 8, 1 }, 'P' }, { { 8, 2 }, 'I' }, { { 8, 3 }, 'D' }, { { 8, 4 }, 'R' }, { { 8, 5 }, 'L' }, { { 8, 6 }, 'F' }, { { 9, 1 }, 'P' }, { { 9, 2 }, 'I' }, { { 9, 3 }, 'D' }, { { 9, 4 }, 'R' }, { { 9, 5 }, 'L' }, { { 9, 6 }, 'F' }, { { 10, 1 }, 'P' }, { { 10, 2 }, 'I' }, { { 10, 3 }, 'D' }, { { 10, 4 }, 'R' }, { { 10, 5 }, 'L' }, { { 10, 6 }, 'F' }, { { 11, 1 }, 'P' }, { { 11, 2 }, 'I' }, { { 11, 3 }, 'D' }, { { 11, 4 }, 'R' }, { { 11, 5 }, 'L' }, { { 11, 6 }, 'F' } };

CANDriver::CANDriver() {
}

int32_t CANDriver::bytesToInt(uint8_t * bytes) {
    union {
        int32_t i;
        uint8_t b[4];
    } u;
    for( int i = 0; i < 4; i++)
        u.b[i] = bytes[i];
    return u.i;
}

float CANDriver::bytesToFloat(uint8_t * bytes) {
    union {
        float f;
        uint8_t b[4];
    } u;
    for( int i = 0; i < 4; i++)
        u.b[i] = bytes[i];
    return u.f;
}

double CANDriver::bytesToDouble(uint8_t * bytes) {
    union {
        double d;
        uint8_t b[8];
    } u;
    for( int i = 0; i < 8; i++)
        u.b[i] = bytes[i];
    return u.d;
}

unsigned char const * const CANDriver::floatToBytes(float *f) {
    return 0; //(unsigned char const *)&f;
}

/*
 * getBits - get a segment of bits from the index, and return as uint8_t
 */
uint8_t CANDriver::getBits(uint32_t value, uint8_t index) {
    return static_cast<uint8_t>((value >> shr[index] ) & bitMask[index]);
}

void CANDriver::init(int tx, int rx, Commander command) {
    this->command = command;
    this->command.verbose = VerboseMode::nothing;

    // hardware specific call
    _initCAN(tx, rx);
}

void CANDriver::transmit() {
    if(this->stream.dataType != CANStream::dt::none) {
        uint32_t identifier = this->identifier;
        uint8_t data[] = "AABBCCDD";
        switch(this->stream.dataType) {
            case CANStream::dt::double_val:
            break;
            case CANStream::dt::char_val:
            break;
            case CANStream::dt::int_val:
            break;
            default: {}
        }

        // hardware specific call
        _transmitCAN(identifier, data, 8);
    }
}

void CANDriver::receive() {
    uint32_t identifier;
    uint8_t data[8];
    uint8_t length;

    // hardware specific call
    if (_receiveCAN(&identifier, data, &length) ) {
        printf("--- Recieved CAN Frame with ID: %d ---\n", identifier);
        this->identifier = identifier;
        this->stream.dataType = CANStream::dt::none;
        uint8_t busID = getBits(identifier, 0);
        uint8_t operation = getBits(identifier, 1);
        uint8_t dataType = getBits(identifier, 2);
        uint8_t global = getBits(identifier, 3);
        uint8_t command = getBits(identifier, 4);
        uint8_t subCommand = getBits(identifier, 5);
        /*
        printf("BusID: %u\n", busID);
        printf("Operation: %u\n", operation);
        printf("Data Type: %u\n", dataType);
        printf("Global: %u\n", global);
        printf("Command: %u\n", command);
        printf("SubCommand: %u\n", subCommand);
        */
        char textCommand[] = "\0\0\0";

        // Global command
        if( globals.find(global) != globals.end() ) {
            textCommand[0] = globals[global];
        }

        // Command
        if( commands.find(command) != commands.end() ) {
            textCommand[1] = commands[command];
        }

        // SubCommand
        if( subCommands.find(std::make_pair(command, subCommand)) != subCommands.end() )
        {
            textCommand[2] = subCommands[std::make_pair(command, subCommand)];
        }

        switch(dataType) {
            case 0: // Float - 4 bytes
                snprintf ( stream.value_buffer, 100, "%s%f%c", textCommand, bytesToFloat(data), this->command.eol );
                break;
            case 1: // Double - 8 bytes
                snprintf ( stream.value_buffer, 100, "%s%f%c", textCommand, bytesToDouble(data), this->command.eol );
                break;
            case 2: // Unsigned Char - 1 byte
                snprintf ( stream.value_buffer, 100, "%s%c%c", textCommand, data[0], this->command.eol );
                break;
            case 3: // Int - 4 bytes
                snprintf ( stream.value_buffer, 100, "%s%i%c", textCommand, bytesToInt(data), this->command.eol );
                break;
        }
        
        stream.value_position = 0;
        printf("Commander format:\n%s",stream.value_buffer);
    }
}