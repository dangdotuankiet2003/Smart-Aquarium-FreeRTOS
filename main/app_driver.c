#include <string.h>
#include <math.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include "esp_adc_cal.h"
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 
#include <app_reset.h>
#include "app_priv.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "ds18b20.h"
#include "i2c-lcd.h" 

#define UNUSED(x)         (void)(x)
#define DETECTED          1
#define NOT_DETECTED      0

#define TEMP_BUS 4
#define VREF 3.3
#define SCOUNT 30

#define DEBOUNCE_DELAY_MS 100
#define BUTTON_POLL_INTERVAL_MS 50

float g_pHValue;
float g_temperature;
float g_tds;

bool mqtt_connected = false;

static SemaphoreHandle_t adc_mutex = NULL;

static TimerHandle_t servo_reset_timer = NULL;

QueueHandle_t sensor_queue;

static void report_state_to_rainmaker(esp_rmaker_device_t *device, bool state);
static void set_power_state(int switch_id, bool target);

static bool g_power_state_servo = DEFAULT_POWER;
static bool g_power_state_pump1 = DEFAULT_POWER;
static bool g_power_state_pump2 = DEFAULT_POWER;
static bool g_power_state_heater = DEFAULT_POWER;

// I2C master initialization for LCD
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    memcpy(bTab, bArray, iFilterLen * sizeof(int));
    for (int j = 0; j < iFilterLen - 1; j++) {
        for (int i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                int temp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = temp;
            }
        }
    }
    return (iFilterLen & 1) ? bTab[(iFilterLen - 1) / 2] : (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

// Callback to reset servo to initial position (0 degrees)
static void servo_reset_callback(TimerHandle_t xTimer) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, SERVO_DUTY_0_DEG);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);

    g_power_state_servo = false;
    report_state_to_rainmaker(switch_device_servo, g_power_state_servo);
}

void display_task(void *arg) {
    char buffer[20]; 
    enum { DISPLAY_SENSORS, DISPLAY_SWITCHES, DISPLAY_LOW_TEMP_MESSAGE, DISPLAY_WATER_DIRTY_MESSAGE } display_state = DISPLAY_SENSORS;
    SensorData_t data;
    static float last_temp_display = DEFAULT_TEMPERATURE;
    static float last_ph_display = DEFAULT_PH;        
    static float last_tds_display = DEFAULT_TDS;        
    static float last_temp_reported = -1000.0;          
    static float last_ph_reported = -1000.0;             
    static float last_tds_reported = -1000.0;            
    TickType_t last_display_time = xTaskGetTickCount();
    static bool heater_auto_control = false; 

    lcd_init();
    lcd_clear();
    ESP_LOGI("Display Task", "LCD initialized successfully");

    while (1) {
        // Check for sensor data in queue with a short timeout
        while (xQueueReceive(sensor_queue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (data.type) {
                case SENSOR_TEMP:
                    last_temp_display = data.value;
                    break;
                case SENSOR_PH:
                    last_ph_display = data.value;
                    break;
                case SENSOR_TDS:
                    last_tds_display = data.value;
                    break;
                default:
                    break;
            }

            if (last_temp_display < 25.0) {
                if (!g_power_state_heater) {
                    g_power_state_heater = true;
                    set_power_state(4, g_power_state_heater); // Turn on heater
                    report_state_to_rainmaker(switch_device_heater, g_power_state_heater);
                    heater_auto_control = true;
                }
            } else {
                if (g_power_state_heater && heater_auto_control) {
                    g_power_state_heater = false;
                    set_power_state(4, g_power_state_heater); 
                    report_state_to_rainmaker(switch_device_heater, g_power_state_heater);
                    heater_auto_control = false;
                }
            }

            // Report to RainMaker
            switch (data.type) {
                case SENSOR_TEMP:
                    if (fabs(data.value - last_temp_reported) >= 0.1f && mqtt_connected) {
                        last_temp_reported = data.value;
                        esp_err_t ret = esp_rmaker_param_update_and_report(
                            esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
                            esp_rmaker_float(data.value)
                        );
                        if (ret == ESP_OK) {
                            ESP_LOGI("Temperature Report", "Reported: %.2f°C", data.value);
                        } else {
                            ESP_LOGE("Temperature Report", "Failed to report: %d", ret);
                        }
                    }
                    break;
                case SENSOR_PH:
                    if (fabs(data.value - last_ph_reported) >= 0.2f && mqtt_connected) {
                        last_ph_reported = data.value;
                        esp_err_t ret = esp_rmaker_param_update_and_report(
                            esp_rmaker_device_get_param_by_name(ph_sensor_device, "pH"),
                            esp_rmaker_float(data.value)
                        );
                        if (ret == ESP_OK) {
                            ESP_LOGI("pH Report", "Reported: %.2f", data.value);
                        } else {
                            ESP_LOGE("pH Report", "Failed to report: %d", ret);
                        }
                    }
                    break;
                case SENSOR_TDS:
                    if (fabs(data.value - last_tds_reported) >= 5.0f && mqtt_connected) {
                        last_tds_reported = data.value;
                        esp_err_t ret = esp_rmaker_param_update_and_report(
                            esp_rmaker_device_get_param_by_name(tds_sensor_device, "TDS"),
                            esp_rmaker_float(data.value)
                        );
                        if (ret == ESP_OK) {
                            ESP_LOGI("TDS Report", "Reported: %.2f", data.value);
                        } else {
                            ESP_LOGE("TDS Report", "Failed to report: %d", ret);
                        }
                    }
                    break;
                default:
                    break;
            }
        }

        if ((xTaskGetTickCount() - last_display_time) >= pdMS_TO_TICKS(2000)) {
            lcd_clear();

            if (display_state == DISPLAY_SENSORS) {
                // Display sensor data
                sprintf(buffer, "Temp: %.2f C", last_temp_display);
                lcd_put_cur(0, 0);
                lcd_send_string(buffer);

                sprintf(buffer, "pH: %.2f", last_ph_display);
                lcd_put_cur(1, 0);
                lcd_send_string(buffer);

                sprintf(buffer, "TDS: %.2f ppm", last_tds_display);
                lcd_put_cur(2, 0);
                lcd_send_string(buffer);

                lcd_put_cur(3, 0);
                lcd_send_string("Smart Aquarium");

                display_state = DISPLAY_SWITCHES;
            } else if (display_state == DISPLAY_SWITCHES) {
                // Display device states (synchronized with RainMaker)
                sprintf(buffer, "Servo: %s", g_power_state_servo ? "ON" : "OFF");
                lcd_put_cur(0, 0);
                lcd_send_string(buffer);

                sprintf(buffer, "Pump1: %s", g_power_state_pump1 ? "ON" : "OFF");
                lcd_put_cur(1, 0);
                lcd_send_string(buffer);

                sprintf(buffer, "Pump2: %s", g_power_state_pump2 ? "ON" : "OFF");
                lcd_put_cur(2, 0);
                lcd_send_string(buffer);

                sprintf(buffer, "Heater: %s", g_power_state_heater ? "ON" : "OFF");
                lcd_put_cur(3, 0);
                lcd_send_string(buffer);

                if(last_temp_display <= 25.0){
                    display_state = DISPLAY_LOW_TEMP_MESSAGE;
                }
                else if (last_tds_display >= 500.0)
                {
                    display_state = DISPLAY_WATER_DIRTY_MESSAGE;
                }
                else{
                    display_state = DISPLAY_SENSORS;
                }
            } else if (display_state == DISPLAY_LOW_TEMP_MESSAGE) {
                // Display low temperature message
                lcd_put_cur(0, 0);
                lcd_send_string("Low Temperature");

                lcd_put_cur(1, 0);
                lcd_send_string("Auto turn on Heater");

                lcd_put_cur(2, 0);
                lcd_send_string("                ");
                lcd_put_cur(3, 0);
                lcd_send_string("                ");

                if(last_tds_display >= 500.0)
                {
                    display_state = DISPLAY_WATER_DIRTY_MESSAGE;
                }
                else{
                    display_state = DISPLAY_SENSORS;
                }
            } else if (display_state == DISPLAY_WATER_DIRTY_MESSAGE) {
                // Display water dirty message
                lcd_put_cur(0, 0);
                lcd_send_string("Water Dirty");

                lcd_put_cur(1, 0);
                lcd_send_string("Need Water Change");

                // Clear remaining lines
                lcd_put_cur(2, 0);
                lcd_send_string("                ");
                lcd_put_cur(3, 0);
                lcd_send_string("                ");

                display_state = DISPLAY_SENSORS;
            }

            last_display_time = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void sensor_task(void *arg) {
    int analogBuffer[SCOUNT] = {0}; 
    int analogBufferTemp[SCOUNT];
    int analogBufferIndex = 0;
    static float last_temp = -1000.0;
    static float last_ph = -1000.0;
    static float last_tds = -1000.0;
    bool first_run = true; 

    g_temperature = DEFAULT_TEMPERATURE;
    g_pHValue = DEFAULT_PH;
    g_tds = DEFAULT_TDS;

    // Initialize ADC semaphore
    adc_mutex = xSemaphoreCreateMutex();
    if (adc_mutex == NULL) {
        ESP_LOGE("Sensor Task", "Failed to create ADC mutex");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("Sensor Task", "Started sensor task");

    while (1) {
        // Read temperature (DS18B20)
        uint8_t received_data[9] = {0};
        int device_presence = Init();
        if (device_presence == DETECTED) {
            GetData(received_data);
            float temp = DecodeTemp(received_data[0], received_data[1], DS18B20_RESOLUTION_9_BIT);
            if (first_run || fabs(temp - last_temp) >= 0.1f) {
                g_temperature = temp;
                last_temp = g_temperature;
                ESP_LOGI("Temperature Sensor", "Read temperature: %.2f°C", g_temperature);

                // Send temperature data to queue
                SensorData_t data = { .type = SENSOR_TEMP, .value = g_temperature };
                xQueueSend(sensor_queue, &data, 0);
            }
        } else {
            ESP_LOGE("Temperature Sensor", "Device not found, using default");
            g_temperature = DEFAULT_TEMPERATURE;
            if (first_run || fabs(g_temperature - last_temp) >= 0.1f) {
                last_temp = g_temperature;
                // Send default temperature to queue
                SensorData_t data = { .type = SENSOR_TEMP, .value = g_temperature };
                xQueueSend(sensor_queue, &data, 0);
            }
        }

        // Read pH (ADC1_CHANNEL_7)
        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret = adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
            if (ret != ESP_OK) {
                ESP_LOGE("pH Sensor", "Failed to configure ADC channel: %d", ret);
                g_pHValue = DEFAULT_PH;
                if (first_run || fabs(g_pHValue - last_ph) >= 0.2f) {
                    last_ph = g_pHValue;
                    // Send default pH to queue
                    SensorData_t data = { .type = SENSOR_PH, .value = g_pHValue };
                    xQueueSend(sensor_queue, &data, 0);
                }
                xSemaphoreGive(adc_mutex);
            } else {
                ret = adc1_config_width(ADC_WIDTH_BIT_12);
                if (ret != ESP_OK) {
                    ESP_LOGE("pH Sensor", "Failed to configure ADC width: %d", ret);
                    g_pHValue = DEFAULT_PH;
                    if (first_run || fabs(g_pHValue - last_ph) >= 0.2f) {
                        last_ph = g_pHValue;
                        // Send default pH to queue
                        SensorData_t data = { .type = SENSOR_PH, .value = g_pHValue };
                        xQueueSend(sensor_queue, &data, 0);
                    }
                    xSemaphoreGive(adc_mutex);
                } else {
                    int adc_value = adc1_get_raw(ADC1_CHANNEL_7);
                    if (adc_value < 0) {
                        ESP_LOGE("pH Sensor", "ADC read error");
                        g_pHValue = DEFAULT_PH;
                        if (first_run || fabs(g_pHValue - last_ph) >= 0.2f) {
                            last_ph = g_pHValue;
                            // Send default pH to queue
                            SensorData_t data = { .type = SENSOR_PH, .value = g_pHValue };
                            xQueueSend(sensor_queue, &data, 0);
                        }
                        xSemaphoreGive(adc_mutex);
                    } else {
                        float voltage = adc_value * 3.3 / 4096.0;
                        float ph = 7 - (2.5 - voltage) * -5.436;
                        if (first_run || fabs(ph - last_ph) >= 0.2f) { 
                            g_pHValue = ph;
                            last_ph = g_pHValue;
                            ESP_LOGI("pH Sensor", "Read pH: %.2f (ADC: %d, Voltage: %.2fV)", g_pHValue, adc_value, voltage);

                            // Send pH data to queue
                            SensorData_t data = { .type = SENSOR_PH, .value = g_pHValue };
                            xQueueSend(sensor_queue, &data, 0);
                        }
                        xSemaphoreGive(adc_mutex);
                    }
                }
            }
        } else {
            ESP_LOGE("pH Sensor", "Failed to acquire ADC mutex");
            g_pHValue = DEFAULT_PH;
            if (first_run || fabs(g_pHValue - last_ph) >= 0.2f) {
                last_ph = g_pHValue;
                // Send default pH to queue
                SensorData_t data = { .type = SENSOR_PH, .value = g_pHValue };
                xQueueSend(sensor_queue, &data, 0);
            }
        }

        // Read TDS (ADC1_CHANNEL_4)
        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret = adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);
            if (ret != ESP_OK) {
                ESP_LOGE("TDS Sensor", "Failed to configure ADC channel: %d", ret);
                g_tds = DEFAULT_TDS;
                if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                    last_tds = g_tds;
                    // Send default TDS to queue
                    SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                    xQueueSend(sensor_queue, &data, 0);
                }
                xSemaphoreGive(adc_mutex);
            } else {
                ret = adc1_config_width(ADC_WIDTH_BIT_12);
                if (ret != ESP_OK) {
                    ESP_LOGE("TDS Sensor", "Failed to configure ADC width: %d", ret);
                    g_tds = DEFAULT_TDS;
                    if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                        last_tds = g_tds;
                        // Send default TDS to queue
                        SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                        xQueueSend(sensor_queue, &data, 0);
                    }
                    xSemaphoreGive(adc_mutex);
                } else {
                    int adc_value = adc1_get_raw(ADC1_CHANNEL_4);
                    if (adc_value < 0 || adc_value > 4095) { 
                        ESP_LOGE("TDS Sensor", "ADC read error: %d", adc_value);
                        g_tds = DEFAULT_TDS;
                        if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                            last_tds = g_tds;
                            // Send default TDS to queue
                            SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                            xQueueSend(sensor_queue, &data, 0);
                        }
                        xSemaphoreGive(adc_mutex);
                    } else if (adc_value == 0) { 
                        ESP_LOGE("TDS Sensor", "Sensor disconnected or faulty (ADC: 0)");
                        g_tds = DEFAULT_TDS;
                        if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                            last_tds = g_tds;
                            // Send default TDS to queue
                            SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                            xQueueSend(sensor_queue, &data, 0);
                        }
                        xSemaphoreGive(adc_mutex);
                    } else {
                        analogBuffer[analogBufferIndex] = adc_value;
                        analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;

                        for (int i = 0; i < SCOUNT; i++) {
                            analogBufferTemp[i] = analogBuffer[i];
                        }

                        float averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (3.3 / 4096.0);
                        float compensationCoefficient = 1.0 + 0.02 * (g_temperature - 25.0);
                        float compensationVoltage = averageVoltage / compensationCoefficient;

                        if (compensationVoltage < 0 || compensationVoltage > 3.3) {
                            ESP_LOGE("TDS Sensor", "Invalid compensationVoltage: %.2fV", compensationVoltage);
                            g_tds = DEFAULT_TDS;
                            if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                                last_tds = g_tds;
                                // Send default TDS to queue
                                SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                                xQueueSend(sensor_queue, &data, 0);
                            }
                        } else {
                            float tds = (133.42 * pow(compensationVoltage, 3)
                                         - 255.86 * pow(compensationVoltage, 2)
                                         + 857.39 * compensationVoltage) * 0.5;
                            if (tds < 0 || tds > 5000) {
                                ESP_LOGE("TDS Sensor", "Invalid TDS value: %.2f", tds);
                                g_tds = DEFAULT_TDS;
                                if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                                    last_tds = g_tds;
                                    // Send default TDS to queue
                                    SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                                    xQueueSend(sensor_queue, &data, 0);
                                }
                            } else if (first_run || fabs(tds - last_tds) >= 5.0f) { 
                                g_tds = tds;
                                last_tds = g_tds;
                                ESP_LOGI("TDS Sensor", "Read TDS: %.2f (ADC: %d, Voltage: %.2fV)", g_tds, adc_value, averageVoltage);

                                // Send TDS data to queue
                                SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                                xQueueSend(sensor_queue, &data, 0);
                            }
                        }
                        xSemaphoreGive(adc_mutex);
                    }
                }
            }
        } else {
            ESP_LOGE("TDS Sensor", "Failed to acquire ADC mutex");
            g_tds = DEFAULT_TDS;
            if (first_run || fabs(g_tds - last_tds) >= 5.0f) {
                last_tds = g_tds;
                // Send default TDS to queue
                SensorData_t data = { .type = SENSOR_TDS, .value = g_tds };
                xQueueSend(sensor_queue, &data, 0);
            }
        }

        first_run = false; 
        vTaskDelay(pdMS_TO_TICKS(REPORTING_PERIOD * 1000));
    }
}

static void set_power_state(int switch_id, bool target) {
    if (switch_id == 1) {
        if (target) {
            // Rotate 90 angle
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, SERVO_DUTY_90_DEG);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
            if (servo_reset_timer == NULL) {
                servo_reset_timer = xTimerCreate("servo_reset_timer", pdMS_TO_TICKS(5000), pdFALSE, NULL, servo_reset_callback);
            }
            if (servo_reset_timer != NULL) {
                xTimerStart(servo_reset_timer, 0);
            }
        } else {
            // Rotate 0 angle again
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, SERVO_DUTY_0_DEG);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
        }
    } else if (switch_id == 2) {
        gpio_set_level(PUMP1_PIN, target);
    } else if (switch_id == 3) {
        gpio_set_level(PUMP2_PIN, target);
    } else if (switch_id == 4) {
        gpio_set_level(HEATER_PIN, target);
    }
}

static void report_state_to_rainmaker(esp_rmaker_device_t *device, bool state) {
    esp_rmaker_param_t *param = esp_rmaker_device_get_param_by_name(device, ESP_RMAKER_DEF_POWER_NAME);
    if (param && mqtt_connected) {
        esp_err_t ret = esp_rmaker_param_update_and_report(param, esp_rmaker_bool(state));
        if (ret == ESP_OK) {
            ESP_LOGI("RainMaker Report", "Updated state: %s", state ? "ON" : "OFF");
        } else {
            ESP_LOGE("RainMaker Report", "Failed to update state: %d", ret);
        }
    } else {
        ESP_LOGW("RainMaker Report", "Skipped: MQTT not connected or invalid param");
    }
}

void button_task(void *arg) {
    // Initialize button GPIOs
    gpio_config_t btn_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = ((1ULL << BUTTON_SERVO) |
                         (1ULL << BUTTON_PUMP1) |
                         (1ULL << BUTTON_PUMP2) |
                         (1ULL << BUTTON_HEATER))
    };
    esp_err_t ret = gpio_config(&btn_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("Button Task", "Failed to configure button GPIOs: %d", ret);
        vTaskDelete(NULL);
        return;
    }

    bool last_servo_state = !BUTTON_ACTIVE_LEVEL;
    bool last_pump1_state = !BUTTON_ACTIVE_LEVEL;
    bool last_pump2_state = !BUTTON_ACTIVE_LEVEL;
    bool last_heater_state = !BUTTON_ACTIVE_LEVEL;

    // Track reported states to avoid redundant logs
    bool servo_reported = g_power_state_servo;
    bool pump1_reported = g_power_state_pump1;
    bool pump2_reported = g_power_state_pump2;
    bool heater_reported = g_power_state_heater;

    ESP_LOGI("Button Task", "Started button polling task");

    while (1) {
        // Read GPIO states
        bool servo_state = gpio_get_level(BUTTON_SERVO) == BUTTON_ACTIVE_LEVEL;
        bool pump1_state = gpio_get_level(BUTTON_PUMP1) == BUTTON_ACTIVE_LEVEL;
        bool pump2_state = gpio_get_level(BUTTON_PUMP2) == BUTTON_ACTIVE_LEVEL;
        bool heater_state = gpio_get_level(BUTTON_HEATER) == BUTTON_ACTIVE_LEVEL;

        // Handle Servo button
        if (servo_state && !last_servo_state) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
            if (gpio_get_level(BUTTON_SERVO) == BUTTON_ACTIVE_LEVEL) {
                ESP_LOGI("Button Servo", "Physical button pressed");
                bool new_state = !g_power_state_servo;  
                ret = app_driver_set_state(new_state, 1);
                if (ret == ESP_OK) {
                    if (servo_reported != new_state) {
                        report_state_to_rainmaker(switch_device_servo, new_state);
                        servo_reported = new_state;
                    }
                    if (!new_state && servo_reset_timer != NULL) { 
                        xTimerStop(servo_reset_timer, 0);
                    }
                } else {
                    ESP_LOGE("Button Servo", "Failed to set state: %d", ret);
                }
            }
        }

        // Handle Pump1 button
        if (pump1_state && !last_pump1_state) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
            if (gpio_get_level(BUTTON_PUMP1) == BUTTON_ACTIVE_LEVEL) {
                ESP_LOGI("Button Pump1", "Physical button pressed");
                bool new_state = !g_power_state_pump1;
                ret = app_driver_set_state(new_state, 2);
                if (ret == ESP_OK) {
                    if (pump1_reported != new_state) {
                        report_state_to_rainmaker(switch_device_pump1, new_state);
                        pump1_reported = new_state;
                    }
                } else {
                    ESP_LOGE("Button Pump1", "Failed to set state: %d", ret);
                }
            }
        }

        // Handle Pump2 button
        if (pump2_state && !last_pump2_state) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
            if (gpio_get_level(BUTTON_PUMP2) == BUTTON_ACTIVE_LEVEL) {
                ESP_LOGI("Button Pump2", "Physical button pressed");
                bool new_state = !g_power_state_pump2;
                ret = app_driver_set_state(new_state, 3);
                if (ret == ESP_OK) {
                    if (pump2_reported != new_state) {
                        report_state_to_rainmaker(switch_device_pump2, new_state);
                        pump2_reported = new_state;
                    }
                } else {
                    ESP_LOGE("Button Pump2", "Failed to set state: %d", ret);
                }
            }
        }

        // Handle Heater button
        if (heater_state && !last_heater_state) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
            if (gpio_get_level(BUTTON_HEATER) == BUTTON_ACTIVE_LEVEL) {
                ESP_LOGI("Button Heater", "Physical button pressed");
                bool new_state = !g_power_state_heater;
                ret = app_driver_set_state(new_state, 4);
                if (ret == ESP_OK) {
                    if (heater_reported != new_state) {
                        report_state_to_rainmaker(switch_device_heater, new_state);
                        heater_reported = new_state;
                    }
                } else {
                    ESP_LOGE("Button Heater", "Failed to set state: %d", ret);
                }
            }
        }

        last_servo_state = servo_state;
        last_pump1_state = pump1_state;
        last_pump2_state = pump2_state;
        last_heater_state = heater_state;

        static uint32_t last_log_time = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_log_time >= 10000) {
            UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("Button Task", "Stack high water mark: %u bytes", stack_high_water_mark * sizeof(StackType_t));
            last_log_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
    }
}

void app_driver_init() {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = ((1ULL << PUMP1_PIN) | (1ULL << PUMP2_PIN) | (1ULL << HEATER_PIN))
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("Driver Init", "Failed to configure output GPIOs: %d", ret);
    }

    // Configure PWM for servo
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = SERVO_PWM_TIMER
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE("Driver Init", "Failed to configure PWM timer: %d", ret);
        return;
    }

    ledc_channel_config_t ledc_channel = {
        .channel = SERVO_PWM_CHANNEL,
        .duty = SERVO_DUTY_0_DEG, 
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = SERVO_PWM_TIMER
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE("Driver Init", "Failed to configure PWM channel: %d", ret);
        return;
    }

    // Initialize I2C for LCD 
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI("Driver Init", "I2C initialized successfully");

    // Create sensor queue
    sensor_queue = xQueueCreate(SENSOR_QUEUE_SIZE, SENSOR_QUEUE_ITEM_SIZE);
    if (sensor_queue == NULL) {
        ESP_LOGE("Driver Init", "Failed to create sensor queue");
    }
}

int IRAM_ATTR app_driver_set_state(bool state, int switch_id) {
    if (switch_id == 1) {
        g_power_state_servo = state;
        set_power_state(switch_id, state);
    } else if (switch_id == 2 && g_power_state_pump1 != state) {
        g_power_state_pump1 = state;
        set_power_state(switch_id, state);
    } else if (switch_id == 3 && g_power_state_pump2 != state) {
        g_power_state_pump2 = state;
        set_power_state(switch_id, state);
    } else if (switch_id == 4 && g_power_state_heater != state) {
        g_power_state_heater = state;
        set_power_state(switch_id, state);
    }
    return ESP_OK;
}

bool app_driver_get_state(int switch_id) {
    if (switch_id == 1) return g_power_state_servo;
    if (switch_id == 2) return g_power_state_pump1;
    if (switch_id == 3) return g_power_state_pump2;
    if (switch_id == 4) return g_power_state_heater;
    return false;
}