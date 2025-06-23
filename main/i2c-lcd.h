/**
 * i2c-lcd.h
 * Header file for I2C LCD library, compatible with 16x2 and 20x4 LCDs using HD44780 controller with PCF8574 I2C module.
 */

 #ifndef I2C_LCD_H
 #define I2C_LCD_H
 
 void lcd_init(void);        // Initialize LCD
 void lcd_send_cmd(char cmd); // Send command to the LCD
 void lcd_send_data(char data); // Send data (character) to the LCD
 void lcd_send_string(char *str); // Send string to the LCD
 void lcd_put_cur(int row, int col); // Set cursor position: row (0-3 for 20x4), col (0-19 for 20x4)
 void lcd_clear(void);       // Clear LCD display
 
 #endif