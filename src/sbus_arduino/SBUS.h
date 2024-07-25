#ifndef SBUS_H
#define SBUS_H

#include <Arduino.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

class SBUS {
  public:
    SBUS(HardwareSerial& serial) : sbusSerial(serial), failsafe_status(SBUS_SIGNAL_FAILSAFE) {
      memset(channels, 0, sizeof(channels));
      memset(sbus_data, 0, sizeof(sbus_data));
    }

    void begin() {
      sbusSerial.begin(100000, SERIAL_8E2);
    }

    void update() {
      readSBUS();
      decodeSBUS();
    }

    int16_t channels[18];
    uint8_t getFailsafeStatus() const {
      return failsafe_status;
    }

  private:
    HardwareSerial& sbusSerial;
    uint8_t sbus_data[25];
    uint8_t failsafe_status;

    void readSBUS() {
      uint8_t sbus_pointer = 0;

      while (sbusSerial.available()) {
        uint8_t data = sbusSerial.read();
        switch (sbus_pointer) {
          case 0:
            if (data == 0x0f) {
              sbus_data[sbus_pointer] = data;
              sbus_pointer++;
            }
            break;

          case 24:
            if (data == 0x00) {
              sbus_data[sbus_pointer] = data;
              sbus_pointer = 0; // Reset for next packet
              return;
            }
            break;

          default:
            sbus_data[sbus_pointer] = data;
            sbus_pointer++;
        }
      }
    }

    void decodeSBUS() {
      memset(channels, 0, sizeof(channels));

      uint8_t byte_in_sbus = 1;
      uint8_t bit_in_sbus = 0;
      uint8_t ch = 0;
      uint8_t bit_in_channel = 0;

      for (uint8_t i = 0; i < 176; i++) {
        if (sbus_data[byte_in_sbus] & (1 << bit_in_sbus)) {
          channels[ch] |= (1 << bit_in_channel);
        }
        bit_in_sbus++;
        bit_in_channel++;

        if (bit_in_sbus == 8) {
          bit_in_sbus = 0;
          byte_in_sbus++;
        }
        if (bit_in_channel == 11) {
          bit_in_channel = 0;
          ch++;
        }
      }

      channels[16] = (sbus_data[23] & (1 << 0)) ? 1 : 0;
      channels[17] = (sbus_data[23] & (1 << 1)) ? 1 : 0;

      failsafe_status = SBUS_SIGNAL_OK;
      if (sbus_data[23] & (1 << 2)) {
        failsafe_status = SBUS_SIGNAL_LOST;
      }
      if (sbus_data[23] & (1 << 3)) {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
      }
    }
};

#endif
