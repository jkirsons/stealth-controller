#include "Arduino.h" // Must be here for harware defines to be linked first
#include "../can_api.h"

#if defined(ESP_H)

#include <string.h>
#include "driver/gpio.h"
#include "driver/can.h"

void _initCAN(int tx, int rx) {
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

void _transmitCAN(uint32_t identifier, uint8_t *data, uint8_t length) {
    //Configure message to transmit
    can_message_t message;
    message.identifier = identifier;
    //message.extd = 1;
    message.flags = CAN_MSG_FLAG_EXTD;
    message.data_length_code = length;
    for (int i = 0; i < length; i++) {
        message.data[i] = data[i];
    }

    //Queue message for transmission
    if (can_transmit(&message, pdMS_TO_TICKS(1)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to queue message for transmission\n");
    }
}

bool _receiveCAN(uint32_t *identifier, uint8_t *data, uint8_t *length) {
    can_message_t message;
    if (can_receive(&message, pdMS_TO_TICKS(0)) != ESP_OK) {
        return false; // no message
    }

    if (!(message.flags & CAN_MSG_FLAG_EXTD)) {    // 29 bit
        return false; // 11 bit
    }

    if (!(message.flags & CAN_MSG_FLAG_RTR)) {
        *identifier = message.identifier;
        memcpy(data, message.data, message.data_length_code);
        *length = message.data_length_code;
        return true;
    }
    return false;
}

void _getUniqueID(uint8_t * id) {
	// Get MAC address for WiFi station
	esp_read_mac(id, ESP_MAC_WIFI_STA);
}

#endif