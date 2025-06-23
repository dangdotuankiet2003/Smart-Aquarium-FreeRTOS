#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>
#include <esp_rmaker_common_events.h>
#include <app_network.h>
#include <app_insights.h>
#include "esp_wifi.h" 
#include "app_priv.h"
#include <math.h>

#if CONFIG_FREERTOS_UNICORE
 static const BaseType_t app_cpu = 0;
#else
 static const BaseType_t app_cpu = 1;
#endif

static const char *TAG = "ESP_SMART_AQUARIUM";

esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *ph_sensor_device;
esp_rmaker_device_t *tds_sensor_device;
esp_rmaker_device_t *switch_device_servo;
esp_rmaker_device_t *switch_device_pump1;
esp_rmaker_device_t *switch_device_pump2;
esp_rmaker_device_t *switch_device_heater;

// External MQTT connected flag (defined in app_driver.c)
extern bool mqtt_connected;

static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                          const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %s for %s - %s", val.val.b ? "true" : "false", 
                 esp_rmaker_device_get_name(device), esp_rmaker_param_get_name(param));
        if (device == switch_device_servo) {
            // Only allow turning on servo (rotate 90 degrees), auto-off after 5 seconds
            if (val.val.b) {
                app_driver_set_state(val.val.b, 1);
            }
        } else if (device == switch_device_pump1) {
            app_driver_set_state(val.val.b, 2);
        } else if (device == switch_device_pump2) {
            app_driver_set_state(val.val.b, 3);
        } else if (device == switch_device_heater) {
            app_driver_set_state(val.val.b, 4);
        }
        esp_rmaker_param_update(param, val);
    }
    return ESP_OK;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGE(TAG, "RainMaker Claim Failed.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STARTED:
                ESP_LOGI(TAG, "Local Control Started.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
                ESP_LOGI(TAG, "Local Control Stopped.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                mqtt_connected = true;
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGE(TAG, "MQTT Disconnected.");
                mqtt_connected = false;
                break;
            case RMAKER_MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

void app_main()
{
    esp_rmaker_console_init();
    app_driver_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash due to error: %d", err);
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS flash initialized");

    app_network_init();

    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_LOGI(TAG, "Event handlers registered");

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Smart Aquarium", "Smart Aquarium");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise RainMaker node. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }
    ESP_LOGI(TAG, "RainMaker node initialized");

    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Temperature", NULL, g_temperature);
    esp_rmaker_node_add_device(node, temp_sensor_device);

    ph_sensor_device = esp_rmaker_device_create("pH Level", NULL, NULL);
    esp_rmaker_device_add_param(ph_sensor_device,
        esp_rmaker_param_create("pH", ESP_RMAKER_PARAM_PH, esp_rmaker_float(g_pHValue),
        PROP_FLAG_READ | PROP_FLAG_PERSIST));
    esp_rmaker_node_add_device(node, ph_sensor_device);

    tds_sensor_device = esp_rmaker_device_create("TDS", NULL, NULL);
    esp_rmaker_device_add_param(tds_sensor_device,
        esp_rmaker_param_create("TDS", ESP_RMAKER_PARAM_PH, esp_rmaker_float(g_tds),
        PROP_FLAG_READ | PROP_FLAG_PERSIST));
    esp_rmaker_node_add_device(node, tds_sensor_device);

    switch_device_servo = esp_rmaker_device_create("Servo", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(switch_device_servo, write_cb, NULL);
    esp_rmaker_node_add_device(node, switch_device_servo);

    switch_device_pump1 = esp_rmaker_device_create("Pump 1", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(switch_device_pump1, write_cb, NULL);
    esp_rmaker_node_add_device(node, switch_device_pump1);

    switch_device_pump2 = esp_rmaker_device_create("Pump 2", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(switch_device_pump2, write_cb, NULL);
    esp_rmaker_node_add_device(node, switch_device_pump2);

    switch_device_heater = esp_rmaker_device_create("Heater", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_device_add_cb(switch_device_heater, write_cb, NULL);
    esp_rmaker_node_add_device(node, switch_device_heater);

    esp_rmaker_param_t *power_param_servo = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    esp_rmaker_device_add_param(switch_device_servo, power_param_servo);
    esp_rmaker_device_assign_primary_param(switch_device_servo, power_param_servo);

    esp_rmaker_param_t *power_param_pump1 = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    esp_rmaker_device_add_param(switch_device_pump1, power_param_pump1);
    esp_rmaker_device_assign_primary_param(switch_device_pump1, power_param_pump1);

    esp_rmaker_param_t *power_param_pump2 = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    esp_rmaker_device_add_param(switch_device_pump2, power_param_pump2);
    esp_rmaker_device_assign_primary_param(switch_device_pump2, power_param_pump2);

    esp_rmaker_param_t *power_param_heater = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    esp_rmaker_device_add_param(switch_device_heater, power_param_heater);
    esp_rmaker_device_assign_primary_param(switch_device_heater, power_param_heater);

    esp_rmaker_ota_enable_default();
    app_insights_enable();
    esp_rmaker_start();
    ESP_LOGI(TAG, "RainMaker started");

    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start WiFi. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }
    ESP_LOGI(TAG, "WiFi started");

    vTaskDelay(15000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Attempting to create tasks on core %d", app_cpu);

    // Create task for button handling
    TaskHandle_t button_task_handle = NULL;
    BaseType_t result = xTaskCreatePinnedToCore(
        button_task,               
        "button_task",              
        10240,                        
        NULL,                         
        3,                            
        &button_task_handle,         
        app_cpu                       
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task! Error code: %d", result);
    } else {
        ESP_LOGI(TAG, "Button task created successfully on core %d", app_cpu);
    }

    // Create task for sensor handling 
    TaskHandle_t sensor_task_handle = NULL;
    result = xTaskCreatePinnedToCore(
        sensor_task,           
        "sensor_task",         
        10240,                 
        NULL,                  
        3,                     
        &sensor_task_handle,  
        app_cpu                
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task! Error code: %d", result);
    } else {
        ESP_LOGI(TAG, "Sensor task created successfully on core %d", app_cpu);
    }

    // Create task for display 
    TaskHandle_t display_task_handle = NULL;
    result = xTaskCreatePinnedToCore(
        display_task,          
        "display_task",        
        10240,               
        NULL,                  
        3,                     
        &display_task_handle,  
        app_cpu                
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task! Error code: %d", result);
    } else {
        ESP_LOGI(TAG, "Display task created successfully on core %d", app_cpu);
    }

    while (1) {
        vTaskDelay(30000 / portTICK_PERIOD_MS); 
        ESP_LOGI(TAG, "System is running... Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
    }
}