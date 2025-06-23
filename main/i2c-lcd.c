/**
 * i2c-lcd.c
 * Source file for I2C LCD library, compatible with 16x2 and 20x4 LCDs using HD44780 controller with PCF8574 I2C module.
 */

 #include "i2c-lcd.h"
 #include "esp_log.h"
 #include "driver/i2c.h"
 #include "unistd.h"
 
 #define SLAVE_ADDRESS_LCD 0x4E>>1 // I2C address of LCD (0x27 for PCF8574, verify with I2C scanner if needed)
 #define I2C_NUM I2C_NUM_0
 
 static const char *TAG = "LCD";
 
 esp_err_t err;
 
 void lcd_send_cmd(char cmd)
 {
     char data_u, data_l;
     uint8_t data_t[4];
     data_u = (cmd & 0xf0);
     data_l = ((cmd << 4) & 0xf0);
     data_t[0] = data_u | 0x0C; // en=1, rs=0
     data_t[1] = data_u | 0x08; // en=0, rs=0
     data_t[2] = data_l | 0x0C; // en=1, rs=0
     data_t[3] = data_l | 0x08; // en=0, rs=0
     err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
     if (err != 0) ESP_LOGI(TAG, "Error in sending command");
 }
 
 void lcd_send_data(char data)
 {
     char data_u, data_l;
     uint8_t data_t[4];
     data_u = (data & 0xf0);
     data_l = ((data << 4) & 0xf0);
     data_t[0] = data_u | 0x0D; // en=1, rs=1
     data_t[1] = data_u | 0x09; // en=0, rs=1
     data_t[2] = data_l | 0x0D; // en=1, rs=1
     data_t[3] = data_l | 0x09; // en=0, rs=1
     err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
     if (err != 0) ESP_LOGI(TAG, "Error in sending data");
 }
 
 void lcd_clear(void)
 {
     lcd_send_cmd(0x01);
     usleep(5000);
 }
 
 void lcd_put_cur(int row, int col)
 {
     switch (row)
     {
         case 0:
             col |= 0x80; // Row 0: 0x00 + 0x80
             break;
         case 1:
             col |= 0xC0; // Row 1: 0x40 + 0x80
             break;
         case 2:
             col |= 0x94; // Row 2: 0x14 + 0x80 (for 20x4)
             break;
         case 3:
             col |= 0xD4; // Row 3: 0x54 + 0x80 (for 20x4)
             break;
         default:
             col |= 0x80; // Default to row 0 if invalid
             break;
     }
 
     lcd_send_cmd(col);
 }
 
 void lcd_init(void)
 {
     // 4-bit initialization
     usleep(50000); // Wait for >40ms
     lcd_send_cmd(0x30);
     usleep(5000); // Wait for >4.1ms
     lcd_send_cmd(0x30);
     usleep(200); // Wait for >100us
     lcd_send_cmd(0x30);
     usleep(10000);
     lcd_send_cmd(0x20); // Set 4-bit mode
     usleep(10000);
 
     // Display initialization
     lcd_send_cmd(0x28); // Function set: 4-bit, 2 lines (works for 20x4), 5x8 font
     usleep(1000);
     lcd_send_cmd(0x08); // Display off
     usleep(1000);
     lcd_send_cmd(0x01); // Clear display
     usleep(1000);
     usleep(1000);
     lcd_send_cmd(0x06); // Entry mode: increment cursor, no shift
     usleep(1000);
     lcd_send_cmd(0x0C); // Display on, cursor off, no blink
     usleep(1000);
 }
 
 void lcd_send_string(char *str)
 {
     while (*str) lcd_send_data(*str++);
 }