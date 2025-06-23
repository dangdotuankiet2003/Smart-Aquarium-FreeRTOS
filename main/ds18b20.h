
/*
 * @file : ds18b20.h
 * @author : Avinashee Tech
 * @brief : header file for DS18B20 sensor   
 */ 

 #ifndef _DS18B20_H_
 #define _DS18B20_H_
 
 
 #include <stdint.h>
 #include "driver/gpio.h"
 #include "esp_log.h"
 
 #define TAG               "ESP_DS18B20"      //tag for monitoring messages
 
 // 1-wire ROM commands
 #define OWB_ROM_SEARCH        0xF0  // Perform Search ROM cycle to identify devices on the bus
 #define OWB_ROM_READ          0x33  // Read device ROM (single device on bus only)
 #define OWB_ROM_MATCH         0x55  // Address a specific device on the bus by ROM
 #define OWB_ROM_SKIP          0xCC  // Address all devices on the bus simultaneously
 #define OWB_ROM_SEARCH_ALARM  0xEC  // Address all devices on the bus with a set alarm flag
 
 #define OWB_ROM_CODE_STRING_LENGTH (17)  // Typical length of OneWire bus ROM ID as ASCII hex string, including null terminator
 
 
 // 1-wire Function commands
 #define DS18B20_FUNCTION_TEMP_CONVERT       0x44  // Initiate a single temperature conversion
 #define DS18B20_FUNCTION_SCRATCHPAD_WRITE   0x4E  // Write 3 bytes of data to the device scratchpad at positions 2, 3 and 4
 #define DS18B20_FUNCTION_SCRATCHPAD_READ    0xBE  // Read 9 bytes of data (including CRC) from the device scratchpad
 #define DS18B20_FUNCTION_SCRATCHPAD_COPY    0x48  // Copy the contents of the scratchpad to the device EEPROM
 #define DS18B20_FUNCTION_EEPROM_RECALL      0xB8  // Restore alarm trigger values and configuration data from EEPROM to the scratchpad
 #define DS18B20_FUNCTION_POWER_SUPPLY_READ  0xB4  // Determine if a device is using parasitic power
 
 //temeperature bit resolution
 enum
 {
     DS18B20_RESOLUTION_INVALID = -1,  // Invalid resolution
     DS18B20_RESOLUTION_9_BIT   = 9,   // 9-bit resolution, LSB bits 2,1,0 undefined
     DS18B20_RESOLUTION_10_BIT  = 10,  // 10-bit resolution, LSB bits 1,0 undefined
     DS18B20_RESOLUTION_11_BIT  = 11,  // 11-bit resolution, LSB bit 0 undefined
     DS18B20_RESOLUTION_12_BIT  = 12,  // 12-bit resolution (default)
 };
 
 //Function Declarations
 void us_delay(uint32_t time_us);
 void gpio_init(void);
 void WriteBit(uint8_t bit);
 uint8_t ReadBit(void);
 void WriteByte(int data);
 uint8_t ReadByte(void);
 uint8_t CRC_Compute8(uint8_t *buffer, uint8_t length);
 uint8_t _calc_crc(uint8_t crc, uint8_t data);
 uint8_t _calc_crc_block(const uint8_t * buffer, size_t len);
 int Init(void);
 bool isConversionComplete(void);
 void GetData(uint8_t *received_data);
 bool _check_resolution(int resolution);
 float DecodeTemp(uint8_t lsb, uint8_t msb, int resolution);
 
 
 #endif
 