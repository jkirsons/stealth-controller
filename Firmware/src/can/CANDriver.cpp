#include "CANDriver.h"
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

uint32_t bitMask[] = {  0b1111,         //DataType
                        0b11111111,     //Command
                        0b1111,         //MotorID
                        0b111111111111  //BusID
                    };

int shr[] = { 24, 16, 12, 0 }; // shift right by bits

// Map identifier bit segments to commander codes.  Structures are generated here:
// https://docs.google.com/spreadsheets/d/1VWHshxOOnxZyKf5IjVzg1VQMbh7cPu5BULljX7UZJ8g/edit#gid=0

std::map<int,std::vector<char>> commands { { 0, { ' ' } }, { 1, { 'E' } }, { 2, { 'L', 'C' } }, { 3, { 'L', 'U' } }, { 4, { 'L', 'V' } }, { 5, { 'C' } }, { 6, { 'C', 'D' } }, { 7, { 'T' } }, { 8, { 'S' } }, { 9, { 'S', 'M' } }, { 10, { 'S', 'E' } }, { 11, { 'M' } }, { 12, { 'M', 'D' } }, { 13, { 'M', 'C' } }, { 14, { 'M', 'G' } }, { 15, { 'M', 'S' } }, { 16, { 'R' } }, { 17, { 'D', 'P' } }, { 18, { 'D', 'I' } }, { 19, { 'D', 'D' } }, { 20, { 'D', 'R' } }, { 21, { 'D', 'L' } }, { 22, { 'D', 'F' } }, { 23, { 'Q', 'P' } }, { 24, { 'Q', 'I' } }, { 25, { 'Q', 'D' } }, { 26, { 'Q', 'R' } }, { 27, { 'Q', 'L' } }, { 28, { 'Q', 'F' } }, { 29, { 'V', 'P' } }, { 30, { 'V', 'I' } }, { 31, { 'V', 'D' } }, { 32, { 'V', 'R' } }, { 33, { 'V', 'L' } }, { 34, { 'V', 'F' } }, { 35, { 'A', 'P' } }, { 36, { 'A', 'I' } }, { 37, { 'A', 'D' } }, { 38, { 'A', 'R' } }, { 39, { 'A', 'L' } }, { 40, { 'A', 'F' } }, { 41, { 'Z', 'A' } }, { 42, { 'Z', 'S' } }, { 43, { 'Z', 'L' } }, { 240, { '?' } }, { 241, { 'Z', 'M' } }, { 242, { 'Z', 'B' } }, { 243, { 'Z', 'R' } } };
std::map<int,char> motors { { 0, ' ' }, { 1, 'M' }, { 2, 'N' }, { 3, 'O' }, { 4, 'P' }, { 5, 'Q' }, { 6, 'R' }, { 7, 'S' } };

CANDriver::CANDriver(int tx, int rx) {
    this->command.verbose = VerboseMode::on_request;

    // hardware specific call
    _initCAN(tx, rx);
    _getUniqueID(uniqueId);
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

uint8_t const * const CANDriver::floatToBytes(float *f) {
    return (unsigned char const *)f;
}
uint8_t const * const CANDriver::doubleToBytes(double *d) {
    return (unsigned char const *)d;
}
uint8_t const * const CANDriver::intToBytes(int32_t *i) {
    return (unsigned char const *)i;
}

/*
 * getBits - get a segment of bits from the index, and return as uint8_t
 */
uint8_t CANDriver::getBits(uint32_t value, uint8_t index) {
    return static_cast<uint8_t>((value >> shr[index] ) & bitMask[index]);
}

void CANDriver::transmit() {
    if(stream.dataType != stream.none) {
        uint32_t identifier = this->identifier;
        //printf("--- Transmitting CAN Frame with ID: %d ---\n", identifier);
        uint8_t data[] = {0,0,0,0,0,0,0,0};
        identifier &= ~(bitMask[0] << shr[0]); // clear bits
        switch(stream.dataType) {
            case CANStream::dt::double_val:
                //printf("Double Value: %f\n", stream.double_value);
                // Data Type: 2
                identifier |= (2 << shr[0]);
                memcpy(data, doubleToBytes(&stream.double_value), 8);
                break;
            case CANStream::dt::char_val:
                //printf("Char Value: %c\n", stream.char_value);
                // Data Type: 3
                identifier |= (3 << shr[0]);
                data[0] = stream.char_value;
                break;
            case CANStream::dt::int_val:
                //printf("Int Value: %d\n", stream.int_value);
                // Data Type: 4
                identifier |= (4 << shr[0]);
                memcpy(data, intToBytes(&stream.int_value), 4);
                break;
            default: {}
        }

        // hardware specific call
        _transmitCAN(identifier, data, 8);
        stream.dataType = stream.none;
    }
}

void CANDriver::receive() {
    adminTasks();

    uint32_t identifier;
    uint8_t data[8];
    uint8_t length;

    // hardware specific call
    if (_receiveCAN(&identifier, data, &length) ) {
        //printf("--- Recieved CAN Frame with ID: %d ---\n", identifier);
        uint8_t dataType = getBits(identifier, 0);
        uint8_t command = getBits(identifier, 1);
        uint8_t motorID = getBits(identifier, 2);
        uint8_t busID = getBits(identifier, 3);

        // Is this a "Set Bus Id" command?
        if(nodeId == 0 && command == 0xF2 && dataType == 8) {
            if(data[0] == uniqueId[0] && data[1] == uniqueId[1] && data[2] == uniqueId[2] &&
                data[3] == uniqueId[3] && data[4] == uniqueId[4] && data[5] == uniqueId[5])
                nodeId = busID;
            else
                return;
        }

        // Don't continue if the message is not for us
        if(busID != nodeId)
            return;

        stream.dataType = stream.none;
        this->identifier = identifier;
        char textCommand[] = "\0\0\0\0";
        int textCommandPosition = 0;

        // Motor
        if( motors.find(motorID) != motors.end() && motors[motorID] != ' ' )
        {
            textCommand[textCommandPosition] = motors[motorID];
            textCommandPosition++;
        }

        // Command
        if( commands.find(command) != commands.end() ) {
            std::vector<char> com_txt = commands[command];
            for(char c : com_txt) {
                if(c != ' ') {
                    textCommand[textCommandPosition] = c;
                    textCommandPosition++;
                }
            }
        }

        switch(dataType) {
            case 0: // Get value
                snprintf ( stream.value_buffer, 100, "%s%c", textCommand,this->command.eol );
                break;
            case 1: // Float - 4 bytes
                snprintf ( stream.value_buffer, 100, "%s%f%c", textCommand, bytesToFloat(data), this->command.eol );
                break;
            case 2: // Double - 8 bytes
                snprintf ( stream.value_buffer, 100, "%s%f%c", textCommand, bytesToDouble(data), this->command.eol );
                break;
            case 3: // Unsigned Char - 1 byte
                snprintf ( stream.value_buffer, 100, "%s%c%c", textCommand, data[0], this->command.eol );
                break;
            case 4: // Int - 4 bytes
                snprintf ( stream.value_buffer, 100, "%s%i%c", textCommand, bytesToInt(data), this->command.eol );
                break;
        }

        stream.value_position = 0;
        //printf("Commander format:\n%s",stream.value_buffer);
    }
}

void CANDriver::adminTasks() {

    // If we have no busId, then ask to get a NodeId
    if(nodeId == 0 && millis() > lastAdminTime + busIdRequestInterval) {
        lastAdminTime = millis();

        uint32_t id = 0x08F10000;
        uint8_t data[] = {0,0,0,0,0,0,0,0};
        memcpy(data, uniqueId, 6);
        memcpy(data+6, versionId, 2);

        // hardware specific call
        _transmitCAN(id, data, 8);
    }

}