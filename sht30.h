#pragma once
#include <stdint.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>


#define SHT30_I2C_WRITE(addr, buff, len) i2c_master_write_to_device(I2C_NUM_0, addr, buff, len, pdMS_TO_TICKS(100))
#define SHT30_I2C_READ(addr, buff, len) i2c_master_read_from_device(I2C_NUM_0, addr, buff, len, pdMS_TO_TICKS(100))

enum class SHT30Error : int {
    OK = 0,
    I2C_WRITE_FAIL,
    I2C_READ_FAIL,
    CRC_FAIL
};

// returns crc checksum for STH30 I2C serial
inline uint8_t crc_checksum(const uint8_t *data, int len){
    uint8_t crc = 0xFF; //starting remainder per the datasheet
    for (int bit = 0; bit < len; bit++){
        crc ^= data[bit]; 
        for (int jj = 0; jj < 8; jj++){
            uint8_t msb = crc & 0x80;
            crc <<= 1;
            if (msb) crc ^= 0x31;
        }
    }
    return crc;
}

template<uint8_t I2C_ADDR = 0x44>
class SHT30{
    public:
        SHT30Error begin(){
            return SHT30Error::OK;
        }

        SHT30Error read(float &temperature, float &humidity){
            uint8_t cmd[2] = {0x2C, 0x06};
            if (SHT30_I2C_WRITE(I2C_ADDR, cmd, 2) != 0) {
                return SHT30Error::I2C_WRITE_FAIL;
            }
        
            vTaskDelay(pdMS_TO_TICKS(20));

            uint8_t raw[6];
            if (SHT30_I2C_READ(I2C_ADDR, raw, 6) != 0){
                return SHT30Error::I2C_READ_FAIL;
            }
            if (crc_checksum(raw, 2) != raw[2] || crc_checksum(raw+3, 2) != raw[5]){
                return SHT30Error::CRC_FAIL;
            }

            uint16_t temp_raw = (raw[0] << 8) | raw[1];
            uint16_t humidity_raw = (raw[3] << 8) | raw[4];
            temperature = -45.0f + 175.0f * (float)temp_raw / 65535.0f;
            humidity = 100.0f * (float)humidity_raw / 65535.0f;
            return SHT30Error::OK;
        }
};