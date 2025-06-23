/*
 * @file : ds18b20.c
 * @author : Avinashee Tech
 * @brief : source file for DS18B20 sensor   
 */ 

 #include <rom/ets_sys.h>     //for ets_delay_us function
 #include "freertos/FreeRTOS.h"
 #include "string.h"
 
 #include "ds18b20.h"
 
 
 #define GPIO_DS18B20_PIN  (GPIO_NUM_4)      //sensor gpio pin
 #define DEBUG_MSG         0                 //switch to 1 to display messages for more info 
 
 //standard timing in us 
 const int onewire_timing[] = {                
   6,     //A write "1" master pull DQ low duration
   64,    //B write "1" master pull DQ high duration
   60,    //C write "0" master pull DQ low duration
   10,    //D write "0" master pull DQ high duration
   9,     //E read master pull DQ high duration
   55,    //F complete read timeslot + 10ms recovery
   0,     //G wait before reset
   480,   //H master pull DQ low duration
   70,    //I master pull DQ high duration
   420    //J complete presence timeslot + recovery
 };
 
 //timing index 
 enum{
	 A,    
	 B,
	 C,
	 D,
	 E,
	 F,
	 G,
	 H,
	 I,
	 J
 };
 
 //zero-initialize the config structure.
 gpio_config_t ds18b20_pin_conf = {};
 
 /***********************************************************************
  *@brief : microsecond delay function 
  *@param : timing in microseconds
  *@retval : None
 ************************************************************************/
 void us_delay(uint32_t time_us)
 {
	 ets_delay_us(time_us);
 }
 
 /***********************************************************************
  *@brief : gpio init function 
  *@param : None
  *@retval : None
  *@note : configures the gpio pin. 
 ************************************************************************/
 void gpio_init(void){    
	 //disable interrupt
	 ds18b20_pin_conf.intr_type = GPIO_INTR_DISABLE;
	 //set as output mode
	 ds18b20_pin_conf.mode = GPIO_MODE_OUTPUT;
	 //bit mask of the pins that you want to set,e.g.GPIO18/19
	 ds18b20_pin_conf.pin_bit_mask = BIT64(GPIO_DS18B20_PIN);
	 //disable pull-down mode
	 ds18b20_pin_conf.pull_down_en = 0;
	 //disable pull-up mode
	 ds18b20_pin_conf.pull_up_en = 0;
	 //configure GPIO with the given settings
	 gpio_config(&ds18b20_pin_conf);
 }
 
 
 
 /***********************************************************************
  *@brief : 1-wire write bit function 
  *@param : bit to write
  *@retval : None
  *@note : provide 10us recovery time.
 ************************************************************************/
 void WriteBit(uint8_t bit)
 {
	 portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
	 portENTER_CRITICAL(&timeCriticalMutex);
 
	 gpio_set_direction(GPIO_DS18B20_PIN,GPIO_MODE_OUTPUT);
	 if(bit==1){       //if bit to send is high
		 gpio_set_level(GPIO_DS18B20_PIN, 0);  //pull DQ low
		 us_delay(onewire_timing[A]);
		 gpio_set_level(GPIO_DS18B20_PIN, 1);  //pull DQ high
		 us_delay(onewire_timing[B]);  //Complete the time slot and 10us recovery
	 }else{            //if bit to send is low
		 gpio_set_level(GPIO_DS18B20_PIN, 0);  //pull DQ low
		 us_delay(onewire_timing[C]);
		 gpio_set_level(GPIO_DS18B20_PIN, 1);  //pull DQ high
		 us_delay(onewire_timing[D]);
	 }
 
	 portEXIT_CRITICAL(&timeCriticalMutex);
 }
 
 
 /***********************************************************************
  *@brief : 1-wire read bit function 
  *@param : None
  *@retval : bit read
  *@note : provide 10us recovery time.
 ************************************************************************/
 uint8_t ReadBit(void)
 {
	 portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
	 portENTER_CRITICAL(&timeCriticalMutex);
 
	 gpio_set_direction(GPIO_DS18B20_PIN,GPIO_MODE_OUTPUT);
	 gpio_set_level(GPIO_DS18B20_PIN, 0);  //pull DQ low
	 us_delay(onewire_timing[A]);
	 gpio_set_level(GPIO_DS18B20_PIN, 1);  //pull DQ high
	 us_delay(onewire_timing[E]);
	 gpio_set_direction(GPIO_DS18B20_PIN,GPIO_MODE_INPUT);
	 uint8_t bit_value = (uint8_t)gpio_get_level(GPIO_DS18B20_PIN);  //read bit
	 us_delay(onewire_timing[F]);  //Complete the time slot and 10us recovery
 
	 portEXIT_CRITICAL(&timeCriticalMutex);
 
	 return bit_value;
 
 }
 
 
 /***********************************************************************
  *@brief : 1-wire write byte function 
  *@param : byte to write
  *@retval : None
  *@note : calls writebit function for each bit of byte.
 ************************************************************************/
 void WriteByte(int data)
 {
 
	 for(int bit_value=0;bit_value<8;bit_value++){
		 WriteBit(data&0x01);  //least significant bit first
		 data = data >> 1;  // shift the data byte for the next bit
	 }
 }
 
 
 /***********************************************************************
  *@brief : 1-wire read byte function 
  *@param : None
  *@retval : byte read
  *@note : calls readbit function for each bit of byte.
 ************************************************************************/
 uint8_t ReadByte(void)
 {
 
	 uint8_t data = 0;
	 for(int bit_value=0;bit_value<8;bit_value++){
		 int bit_recv = ReadBit();  
		 data = data | ((bit_recv&0x01) << bit_value);  //read bit and shift from least significant bit first
	 }
	 return data;
 }
 
 /***********************************************************************
  *@brief : crc8 computation function 
  *@param : input buffer for crc calculation, length of buffer in bytes
  *@retval : crc value calculated
  *@note : 1st method for crc calculation in this application.
 ************************************************************************/
 uint8_t CRC_Compute8(uint8_t *buffer, uint8_t length)
 {
	 uint8_t crc8 = 0, valid = 0;
	 uint8_t inByte, byteCount, bitCount, mix;
 
	 for (byteCount = 0; byteCount < length; byteCount++)
	 {
		 inByte = buffer[byteCount];
 #if DEBUG_MSG
		 ESP_LOGI(TAG,"buffer-%d",inByte);
 #endif        
		 if (inByte)
		 {
			 valid = 1;
		 }
		 for (bitCount = 0; bitCount < 8; bitCount++)
		 {
			 mix = (crc8 ^ inByte) & 0x01;
			 crc8 >>= 1;
			 if (mix)
			 {
				 crc8 ^= 0x8C;
			 }
			 inByte >>= 1;
		 }
 #if DEBUG_MSG
		 ESP_LOGI(TAG,"crc idx:%d - data:%d",byteCount,crc8);
 #endif
	 }
	 if (!valid)
	 {
		 /* If all bytes are 0, return a different CRC so that the test will fail */
		 return 0xFF;
	 }
	 return crc8;
 }
 
 /***********************************************************************
  *@brief : crc8 lookup table function 
  *@param : previous crc value, current data
  *@retval : updated crc value
  *@note : table for faster computation of crc.
 ************************************************************************/
 uint8_t _calc_crc(uint8_t crc, uint8_t data)
 {
	 static const uint8_t table[256] = {
			 0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
			 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
			 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
			 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
			 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
			 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
			 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
			 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
			 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
			 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
			 175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
			 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
			 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
			 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
			 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
			 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
	 };
 
	 return table[crc ^ data];
 }
 
 /***********************************************************************
  *@brief : crc8 computation function 
  *@param : input buffer for crc calculation, length of buffer in bytes
  *@retval : crc value calculated
  *@note : 2nd method for crc calculation in this application.
 ************************************************************************/
 uint8_t _calc_crc_block(const uint8_t * buffer, size_t len)
 {
	 uint8_t crc_next = 0;
	 uint8_t crc_prev = 0;
	 do
	 {
		 crc_next = _calc_crc(crc_prev, *buffer++);
		 crc_prev = crc_next;
 #if DEBUG_MSG
		 ESP_LOGI(TAG,"crc idx:%d - data:%d",(8-len),crc_next);
 #endif
	 }
	 while (--len > 0);
	 return crc_next;
 }
 
 /***********************************************************************
  *@brief : sensor init function 
  *@param : None
  *@retval : device presence status
  *@note : generate a 1-Wire reset, return 1 if presence detected,
   0 otherwise.
 ************************************************************************/
 int Init(void)
 {
	 portMUX_TYPE timeCriticalMutex = portMUX_INITIALIZER_UNLOCKED;
	 portENTER_CRITICAL(&timeCriticalMutex);
 
	 //reset
	 gpio_set_direction(GPIO_DS18B20_PIN,GPIO_MODE_OUTPUT);
	 us_delay(onewire_timing[G]);
	 gpio_set_level(GPIO_DS18B20_PIN, 0);  //pull DQ low
	 us_delay(onewire_timing[H]);
	 gpio_set_level(GPIO_DS18B20_PIN, 1);  //pull DQ high
	 us_delay(onewire_timing[I]);
	 //presence
	 gpio_set_direction(GPIO_DS18B20_PIN,GPIO_MODE_INPUT);
	 int presence1 = gpio_get_level(GPIO_DS18B20_PIN);  
	 us_delay(onewire_timing[J]);
	 int presence2 = gpio_get_level(GPIO_DS18B20_PIN);  
	 int presence = (presence1==0) && (presence2==1);   //interpret presence of device
 
	 portEXIT_CRITICAL(&timeCriticalMutex);
	 return presence;
 
 }
 
 /***********************************************************************
  *@brief : function to check conversion status 
  *@param : None
  *@retval : conversion status
  *@note : transmitting 0 while the temperature conversion is in progress 
   and 1 when the conversion is done. 
 ************************************************************************/
 bool isConversionComplete(void) 
 {
	 uint8_t conv_status = ReadBit();
	 return (conv_status == 1);    
 }
 
 /***********************************************************************
  *@brief : sensor data function 
  *@param : pointer to buffer for receiving data
  *@retval : None
  *@note : send 1-wire commands and receive temeprature data from sensor.
 ************************************************************************/
 void GetData(uint8_t *received_data)
 {
 
	 //send temperature conversion command
	 Init();
	 WriteByte(OWB_ROM_SKIP);  
	 WriteByte(DS18B20_FUNCTION_TEMP_CONVERT);
	 while(!isConversionComplete()){
		 __asm__ __volatile__ ("nop");
	 }
	 //send scratchpad memory read command
	 Init();
	 WriteByte(OWB_ROM_SKIP);
	 WriteByte(DS18B20_FUNCTION_SCRATCHPAD_READ);    
	 for(int loop=0;loop<9;loop++){
		 received_data[loop] = (uint8_t)ReadByte();
	 }
	 Init();
 
 #if DEBUG_MSG
	 ESP_LOGI(TAG,"Scratchpad memory");
	 for(int loop=0;loop<9;loop++){
		 ESP_LOGI(TAG,"idx:%d-data:%d",loop,received_data[loop]);
	 }
 #endif
 
	 //crc calculation and comparing with received crc
	 uint8_t crc_calculated = 0;
	 uint8_t received_data_copy[9] = {0};
	 memcpy(received_data_copy,received_data,sizeof(received_data_copy));
 
	 crc_calculated = (uint8_t)_calc_crc_block(received_data_copy,8);
	 ESP_LOGI(TAG,"CRC calc:%d",crc_calculated);
	 if(crc_calculated!=received_data_copy[8]){
		 ESP_LOGI(TAG,"CRC incorrect");
		 ESP_LOGI(TAG,"CRC received:%d",received_data_copy[8]);
	 }else{
			 ESP_LOGI(TAG,"CRC correct");
	 }
	 
	 
 }
 
 /***********************************************************************
  *@brief : function to check data resolution witihin range 
  *@param : None
  *@retval : bit resolution range status 
  *@note : 1 if the selected resolution allowed, 0 otherwise
 ************************************************************************/
 bool _check_resolution(int resolution)
 {
	 return (resolution >= DS18B20_RESOLUTION_9_BIT) && (resolution <= DS18B20_RESOLUTION_12_BIT);
 }
 
 
 /***********************************************************************
  *@brief : function to decode received temperature
  *@param : variable to store lower byte, upper byte, temperature bit 
   resolution selected 
  *@retval : temperature data
  *@note : removes bit from the LSB and MSB temperature data based on 
   resolution selected to calculate correct temeperature 
 ************************************************************************/
 float DecodeTemp(uint8_t lsb, uint8_t msb, int resolution)
 {
	 float result = 0.0f;
	 if (_check_resolution(resolution))
	 {
		 // masks to remove undefined bits from result
		 static const uint8_t lsb_mask[4] = { ~0x07, ~0x03, ~0x01, ~0x00 };
		 uint8_t lsb_masked = lsb_mask[resolution - DS18B20_RESOLUTION_9_BIT] & lsb;
		 int16_t raw = (msb << 8) | lsb_masked;
		 result = raw / 16.0f;
	 }
	 else
	 {
		 return 0;
	 }
 
	 return result;
 }