#include "can.h"

#if defined(ESP_H)
/*
void CANDriver::init(int tx, int rx) {
  //Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  //Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      printf("Driver installed\n");
  } else {
      printf("Failed to install driver\n");
      return;
  }

  //Start TWAI driver
  if (twai_start() == ESP_OK) {
      printf("Driver started\n");
  } else {
      printf("Failed to start driver\n");
      return;
  }
}

void CANDriver::transmit() {
  //Configure message to transmit
  twai_message_t message;
  message.identifier = 0xAAAA;
  message.extd = 1;
  message.data_length_code = 4;
  for (int i = 0; i < 4; i++) {
      message.data[i] = 0;
  }

  //Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
      printf("Message queued for transmission\n");
  } else {
      printf("Failed to queue message for transmission\n");
  }
}

void CANDriver::receive() {
  //Wait for message to be received
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
      printf("Message received\n");
  } else {
      printf("Failed to receive message\n");
      return;
  }

  //Process received message
  if (message.extd) {
      printf("Message is in Extended Format\n");
  } else {
      printf("Message is in Standard Format\n");
  }
  printf("ID is %d\n", message.identifier);
  if (!(message.rtr)) {
      for (int i = 0; i < message.data_length_code; i++) {
          printf("Data byte %d = %d\n", i, message.data[i]);
      }
  }
}
*/
#endif