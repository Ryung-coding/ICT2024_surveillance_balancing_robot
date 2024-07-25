#include <Arduino.h>
#include "SBUS.h"

struct PACKET {
  int16_t ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16;
  uint8_t sbus_signal;
};

SBUS my_sbus(Serial5);
PACKET packet;
size_t packet_size = sizeof(packet);
uint32_t last_update_time = 0;
const uint32_t CHANNEL_UPDATING_INTERVAL = 20; // 50Hz

void update_SBUS_Packet(PACKET& packet) {
  packet.ch1 = my_sbus.channels[0];
  packet.ch2 = my_sbus.channels[1];
  packet.ch3 = my_sbus.channels[2];
  packet.ch4 = my_sbus.channels[3];
  packet.ch5 = my_sbus.channels[4];
  packet.ch6 = my_sbus.channels[5];
  packet.ch7 = my_sbus.channels[6];
  packet.ch8 = my_sbus.channels[7];
  packet.ch9 = my_sbus.channels[8];
  packet.ch10 = my_sbus.channels[9];
  packet.ch11 = my_sbus.channels[10];
  packet.ch12 = my_sbus.channels[11];
  packet.ch13 = my_sbus.channels[12];
  packet.ch14 = my_sbus.channels[13];
  packet.ch15 = my_sbus.channels[14];
  packet.ch16 = my_sbus.channels[15];
  packet.sbus_signal = my_sbus.getFailsafeStatus();
}

void sendToPC(const PACKET& packet) {
  uint8_t buffer[packet_size + 2];
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  memcpy(buffer + 2, &packet, packet_size);
  Serial.write(buffer, packet_size + 2);
}

void setup() {
  Serial.begin(115200);
  my_sbus.begin();
  memset(&packet, 0, packet_size); // 패킷 초기화
}

void loop() {
  if (millis() - last_update_time >= CHANNEL_UPDATING_INTERVAL) {
    my_sbus.update();
    update_SBUS_Packet(packet);
    sendToPC(packet);
    last_update_time = millis();
  }
}
