#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_POWER  false
#define BUTTON_ACTIVE_LEVEL  0  

extern float g_temperature; 
extern float g_pHValue;      
extern float g_tds; 
#define DEFAULT_TEMPERATURE 25.0
#define DEFAULT_PH 7.0
#define DEFAULT_TDS 0
#define REPORTING_PERIOD    10 

// GPIO definitions for buttons
#define BUTTON_SERVO     5     
#define BUTTON_PUMP1     12    
#define BUTTON_PUMP2     13    
#define BUTTON_HEATER    27   

#define SERVO_PIN        14
#define PUMP1_PIN        15
#define PUMP2_PIN        16
#define HEATER_PIN       17

#define SERVO_PWM_CHANNEL   LEDC_CHANNEL_0
#define SERVO_PWM_TIMER     LEDC_TIMER_0
#define SERVO_PWM_FREQ_HZ   50  
#define SERVO_PWM_RESOLUTION LEDC_TIMER_10_BIT  
#define SERVO_DUTY_0_DEG    51   
#define SERVO_DUTY_90_DEG   128  

#define SERVO_RESET_DELAY_MS 5000

#define I2C_MASTER_SCL_IO           GPIO_NUM_22      
#define I2C_MASTER_SDA_IO           GPIO_NUM_21      
#define I2C_MASTER_NUM              0                
#define I2C_MASTER_FREQ_HZ          400000           
#define I2C_MASTER_TX_BUF_DISABLE   0                
#define I2C_MASTER_RX_BUF_DISABLE   0               
#define I2C_MASTER_TIMEOUT_MS       1000            

#define SENSOR_QUEUE_SIZE 10  
#define SENSOR_QUEUE_ITEM_SIZE sizeof(SensorData_t)

typedef enum {
    SENSOR_TEMP, 
    SENSOR_PH,
    SENSOR_TDS
} SensorType_t;

typedef struct {
    SensorType_t type;
    float value;
} SensorData_t;

// RainMaker device 
extern esp_rmaker_device_t *temp_sensor_device;
extern esp_rmaker_device_t *ph_sensor_device;
extern esp_rmaker_device_t *tds_sensor_device;
extern esp_rmaker_device_t *switch_device_servo;
extern esp_rmaker_device_t *switch_device_pump1;
extern esp_rmaker_device_t *switch_device_pump2;
extern esp_rmaker_device_t *switch_device_heater;

// Queue declaration
extern QueueHandle_t sensor_queue;

// Function declarations
void app_driver_init(void);
int app_driver_set_state(bool state, int switch_id);
bool app_driver_get_state(int switch_id);
void button_task(void *arg);
void sensor_task(void *arg);
void display_task(void *arg); 