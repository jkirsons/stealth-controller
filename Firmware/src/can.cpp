#include "can.h"

//#if defined(ESP_H)
uint32_t bitMask[] = { 0b1111,   //BusID
                        0b1,   //Operation
                        0b111,   //DataType
                        0b1111,   //Global
                        0b1111,   //Command
                        0b1111 }; //SubCommand;

int shr[] = { 16, 15, 12, 8, 4, 0 }; // shift right by bits  

std::map<int,char> globals { { 5, '?' }, { 6, '@' }, { 7, '#' }, { 0, 'A' }, { 1, 'B' }, { 2, 'C' }, { 3, 'D' }, { 4, 'E' }, { 10, 'M' }, { 11, 'N' }, { 12, 'O' }, { 13, 'P' }, { 14, 'Q' }, { 15, 'R' } };
std::map<int,char> commands { { 1, 'E' }, { 2, 'L' }, { 3, 'C' }, { 4, 'T' }, { 5, 'S' }, { 6, 'M' }, { 7, 'R' }, { 8, 'D' }, { 9, 'Q' }, { 10, 'V' }, { 11, 'A' } };
std::map<std::pair<int, int>, char> subCommands { { { 2, 1 }, 'C' }, { { 2, 2 }, 'U' }, { { 2, 3 }, 'V' }, { { 3, 1 }, 'D' }, { { 5, 1 }, 'M' }, { { 5, 2 }, 'E' }, { { 6, 1 }, 'D' }, { { 6, 2 }, 'C' }, { { 6, 3 }, 'G' }, { { 6, 4 }, 'S' }, { { 8, 1 }, 'P' }, { { 8, 2 }, 'I' }, { { 8, 3 }, 'D' }, { { 8, 4 }, 'R' }, { { 8, 5 }, 'L' }, { { 8, 6 }, 'F' }, { { 9, 1 }, 'P' }, { { 9, 2 }, 'I' }, { { 9, 3 }, 'D' }, { { 9, 4 }, 'R' }, { { 9, 5 }, 'L' }, { { 9, 6 }, 'F' }, { { 10, 1 }, 'P' }, { { 10, 2 }, 'I' }, { { 10, 3 }, 'D' }, { { 10, 4 }, 'R' }, { { 10, 5 }, 'L' }, { { 10, 6 }, 'F' }, { { 11, 1 }, 'P' }, { { 11, 2 }, 'I' }, { { 11, 3 }, 'D' }, { { 11, 4 }, 'R' }, { { 11, 5 }, 'L' }, { { 11, 6 }, 'F' } };

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

CANDriver::CANDriver() {

}

void CANDriver::init(int tx, int rx, Commander command) {
    this->command = command;

    //Initialize configuration structures using macro initializers
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx, (gpio_num_t)rx, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_1MBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    //Install TWAI driver
    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("CAN Driver installed\n");
    } else {
        printf("Failed to install CAN driver\n");
        return;
    }

    //Start TWAI driver
    if (can_start() == ESP_OK) {
        printf("CAN Driver started\n");
    } else {
        printf("Failed to start CAN driver\n");
        return;
    }
}

void CANDriver::transmit() {
  //Configure message to transmit
  can_message_t message;
  message.identifier = 0xAAAA;
  //message.extd = 1;
  message.flags = CAN_MSG_FLAG_EXTD;
  message.data_length_code = 4;
  for (int i = 0; i < 4; i++) {
      message.data[i] = 0;
  }

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
      printf("Message queued for transmission\n");
  } else {
      printf("Failed to queue message for transmission\n");
  }
}

float CANDriver::bytesToFloat(uint8_t * bytes) {
    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[3] = bytes[3];
    u.b[2] = bytes[2];
    u.b[1] = bytes[1];
    u.b[0] = bytes[0];
    return u.f;
}

double CANDriver::bytesToDouble(uint8_t * bytes) {
    union {
        double d;
        uint8_t b[8];
    } u;
    u.b[7] = bytes[7];
    u.b[6] = bytes[6];
    u.b[5] = bytes[5];
    u.b[4] = bytes[4];    
    u.b[3] = bytes[3];
    u.b[2] = bytes[2];
    u.b[1] = bytes[1];
    u.b[0] = bytes[0];
    return u.d;
}

unsigned char const * const CANDriver::floatToBytes(float *f) {
    return 0; //(unsigned char const *)&f;
}

uint8_t * CANDriver::getUniqueID() {
    uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    return baseMac;
}


u_char CANDriver::getBits(uint32_t value, u_char index) {
    return static_cast<u_char>((value >> shr[index] ) & bitMask[index]);
}

void CANDriver::receive() {
    can_message_t message;
    if (can_receive(&message, pdMS_TO_TICKS(0)) != ESP_OK) {
        return; // no message
    }

    if (!(message.flags & CAN_MSG_FLAG_EXTD)) {    // 29 bit
        return; // 11 bit
    }

    printf("--- Recieved CAN Frame with ID: %d ---\n", message.identifier);
    if (!(message.flags & CAN_MSG_FLAG_RTR)) {
        u_char busID = getBits(message.identifier, 0);
        u_char operation = getBits(message.identifier, 1);
        u_char dataType = getBits(message.identifier, 2);
        u_char global = getBits(message.identifier, 3);
        u_char command = getBits(message.identifier, 4);
        u_char subCommand = getBits(message.identifier, 5);
        printf("BusID: %u\n", busID);
        printf("Operation: %u\n", operation);
        printf("Data Type: %u\n", dataType);
        printf("Global: %u\n", global);
        printf("Command: %u\n", command);
        printf("SubCommand: %u\n", subCommand);

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

        double payload = bytesToFloat(message.data);
        snprintf ( stream.value_buffer, 100, "%s%f%c", textCommand, payload, this->command.eol );
        stream.value_position = 0;
        printf("Commander format:\n%s",stream.value_buffer);

    }
}

//#endif